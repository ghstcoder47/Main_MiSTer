// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 Dawid Janas
//
// USB capture for the Minimig parallel-port sampler. A worker owns ALSA and
// converts one input to 48 kHz mono. It publishes frames to a single-producer,
// single-consumer ring; only minimig_sampler_poll() in the main loop uses SPI.
// hps_ext.v routes UIO_DMA_WRITE at 0xF600 to usb_sampler.v. Each frame carries
// two identical 16-bit words, left then right. Status bit 14 grants FIFO space.

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <errno.h>
#include <unistd.h>
#include <fcntl.h>
#include <time.h>
#include <math.h>
#include <pthread.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <stdarg.h>
#include <sound/asound.h>

#include "../../spi.h"
#include "../../user_io.h"
#include "../../ide.h"
#include "minimig_config.h"
#include "minimig_sampler.h"

static uint32_t now_ms()
{
	struct timespec ts;
	clock_gettime(CLOCK_MONOTONIC, &ts);
	return (uint32_t)(ts.tv_sec * 1000 + ts.tv_nsec / 1000000);
}

// Debug event log in /tmp, not on SD/USB storage. Begin/end records distinguish
// completed calls from calls that have not returned. Reboots discard the log.
static void slog(const char *fmt, ...)
{
	int saved_errno = errno;
	char line[200];
	va_list ap;
	va_start(ap, fmt);
	vsnprintf(line, sizeof(line), fmt, ap);
	va_end(ap);

	FILE *f = fopen("/tmp/sampler_log.txt", "a");
	if (f && ftell(f) > 65536)
	{
		fclose(f);
		f = fopen("/tmp/sampler_log.txt", "w");
	}
	static bool reported_error;
	int error = f ? 0 : errno;
	if (f)
	{
		if (fprintf(f, "[%u ms pid=%ld] %s", now_ms(), (long)getpid(), line) < 0) error = errno;
		if (fclose(f) && !error) error = errno;
	}
	if (error && !reported_error)
		fprintf(stderr, "Sampler: cannot write event log: %s\n", strerror(error));
	reported_error = error != 0;
	errno = saved_errno;
}

static int capture_ioctl(int fd, unsigned long command, const char *name, void *arg = NULL)
{
	slog("Sampler: fd=%d %s begin\n", fd, name);
	int result = ioctl(fd, command, arg);
	int error = result < 0 ? errno : 0;
	slog("Sampler: fd=%d %s end result=%d errno=%d (%s)\n",
	     fd, name, result, error, error ? strerror(error) : "ok");
	errno = error;
	return result;
}

static void capture_close(int fd)
{
	// ALSA release stops and frees the stream. Extra PCM commands on a lost
	// device are unnecessary; retrying close() could close a reused descriptor.
	slog("Sampler: fd=%d close begin\n", fd);
	int result = close(fd);
	int error = result < 0 ? errno : 0;
	slog("Sampler: fd=%d close end result=%d errno=%d (%s)\n",
	     fd, result, error, error ? strerror(error) : "ok");
}

#define SMP_ADDR        0xF600
#define SMP_STATUS_REQ  0x4000
#define SMP_RATE        48000
#define SMP_BURST       512      // frames per transfer; must fit the FPGA's advertised headroom
#define SMP_MAX_INPUT   16
#define SMP_RING        8192     // frames, power of two

// The worker owns head; the main loop owns tail, including during a flush.
// Counters wrap as unsigned integers. Their distance never exceeds SMP_RING.
// Release/acquire publishes completed frames and releases consumed slots.
static uint32_t ring[SMP_RING];  // (L << 16) | R
static volatile uint32_t ring_head, ring_tail;
static volatile int sampler_wanted;
static pthread_t sampler_thread;
static int thread_started;

// Debug statistics, written and read only by the capture worker (except the
// drop counter, which is also read by the logger on the same thread).
static uint32_t ring_fill_min = ~0u, ring_fill_max;
static uint32_t ring_dropped;

#define SMP_FIR_MAX 256   // enough for captures up to about 256 kHz

#define SMP_TRIM_ABOVE 3072
#define SMP_TRIM_KEEP  1024
static volatile int test_tone;  // Debug: triangle wave, 200 Hz at a 48 kHz capture rate
static volatile int test_hz;    // Debug: if the test file holds a number, a sine of that many Hz

static void ring_put(int16_t l, int16_t r)
{
	uint32_t head = __atomic_load_n(&ring_head, __ATOMIC_RELAXED);
	uint32_t tail = __atomic_load_n(&ring_tail, __ATOMIC_ACQUIRE);
	uint32_t fill = head - tail;
	if (fill < ring_fill_min) ring_fill_min = fill;
	if (fill > ring_fill_max) ring_fill_max = fill;
	// Drop incoming frames rather than overwrite audio the consumer still owns.
	if (fill >= SMP_RING) { ring_dropped++; return; }
	ring[head & (SMP_RING - 1)] = ((uint32_t)(uint16_t)l << 16) | (uint16_t)r;
	__atomic_store_n(&ring_head, head + 1, __ATOMIC_RELEASE);
}

static void mask_set(struct snd_mask *m, unsigned int bit)
{
	memset(m, 0, sizeof(*m));
	m->bits[bit >> 5] = 1u << (bit & 31);
}

static void interval_set(struct snd_interval *i, unsigned int min, unsigned int max)
{
	memset(i, 0, sizeof(*i));
	i->min = min;
	i->max = max;
	i->integer = 1;
}

static struct snd_mask *hw_mask(struct snd_pcm_hw_params *p, int n)
{
	return &p->masks[n - SNDRV_PCM_HW_PARAM_FIRST_MASK];
}

static struct snd_interval *hw_interval(struct snd_pcm_hw_params *p, int n)
{
	return &p->intervals[n - SNDRV_PCM_HW_PARAM_FIRST_INTERVAL];
}

struct capture
{
	int fd;
	int channels;
	int rate;
	int format;
	int sample_bytes;
	unsigned int period;
};

static bool try_params(int fd, int format, int channels, int rate, unsigned int *period)
{
	struct snd_pcm_hw_params p;
	memset(&p, 0, sizeof(p));

	for (int i = 0; i <= SNDRV_PCM_HW_PARAM_LAST_MASK - SNDRV_PCM_HW_PARAM_FIRST_MASK; i++)
		memset(&p.masks[i], 0xff, sizeof(p.masks[i]));

	for (int i = 0; i <= SNDRV_PCM_HW_PARAM_LAST_INTERVAL - SNDRV_PCM_HW_PARAM_FIRST_INTERVAL; i++)
	{
		p.intervals[i].min = 0;
		p.intervals[i].max = ~0u;
	}

	p.rmask = ~0u;
	p.info = ~0u;

	mask_set(hw_mask(&p, SNDRV_PCM_HW_PARAM_ACCESS), SNDRV_PCM_ACCESS_RW_INTERLEAVED);
	mask_set(hw_mask(&p, SNDRV_PCM_HW_PARAM_FORMAT), format);
	mask_set(hw_mask(&p, SNDRV_PCM_HW_PARAM_SUBFORMAT), SNDRV_PCM_SUBFORMAT_STD);
	interval_set(hw_interval(&p, SNDRV_PCM_HW_PARAM_CHANNELS), channels, channels);
	interval_set(hw_interval(&p, SNDRV_PCM_HW_PARAM_RATE), rate, rate);
	interval_set(hw_interval(&p, SNDRV_PCM_HW_PARAM_PERIOD_SIZE), rate / 100, rate / 25);
	interval_set(hw_interval(&p, SNDRV_PCM_HW_PARAM_BUFFER_SIZE), rate / 5, rate);

	// Reject unsupported combinations before asking the driver to configure USB.
	if (ioctl(fd, SNDRV_PCM_IOCTL_HW_REFINE, &p) < 0) return false;
	slog("Sampler: fd=%d configuring %d ch, %d Hz, format=%d\n", fd, channels, rate, format);
	if (capture_ioctl(fd, SNDRV_PCM_IOCTL_HW_PARAMS, "HW_PARAMS", &p) < 0) return false;

	*period = hw_interval(&p, SNDRV_PCM_HW_PARAM_PERIOD_SIZE)->min;
	return true;
}

static bool capture_try_card(int card, struct capture *c)
{
	char path[64];
	snprintf(path, sizeof(path), "/dev/snd/pcmC%dD0c", card);
	slog("Sampler: open %s begin\n", path);
	int fd = open(path, O_RDWR | O_CLOEXEC | O_NONBLOCK);
	int error = fd < 0 ? errno : 0;
	slog("Sampler: open %s end fd=%d errno=%d (%s)\n",
	     path, fd, error, error ? strerror(error) : "ok");
	if (fd < 0)
		return false;

	static const struct { int fmt; int bytes; } formats[] =
	{
		{ SNDRV_PCM_FORMAT_S16_LE, 2 },
		{ SNDRV_PCM_FORMAT_S24_3LE, 3 },
		{ SNDRV_PCM_FORMAT_S32_LE, 4 },
	};
	static const int rates[] = { 48000, 44100, 96000, 88200, 32000, 22050 };

	// Open a wide hardware mode even when only its first 16 inputs are exposed.
	// Some interfaces require more channels than the selected input number.
	for (int ch = 32; ch >= 1; ch--)
	{
		for (int r : rates)
		{
			for (auto &f : formats)
			{
				unsigned int period;
				if (!try_params(fd, f.fmt, ch, r, &period))
				{
					error = errno;
					if (error == EINVAL) continue;
					slog("Sampler: %s parameter search aborted: errno=%d (%s)\n",
					     path, error, strerror(error));
					capture_close(fd);
					return false;
				}
				if (capture_ioctl(fd, SNDRV_PCM_IOCTL_PREPARE, "PREPARE") < 0)
				{
					capture_close(fd);
					return false;
				}

				c->fd = fd;
				c->channels = ch;
				c->rate = r;
				c->format = f.fmt;
				c->sample_bytes = f.bytes;
				c->period = period;
				slog("Sampler: %s, %d ch, %d Hz, %d bytes/sample, period %u\n", path, ch, r, f.bytes, period);
				return true;
			}
		}
	}

	slog("Sampler: %s has no supported capture parameters\n", path);
	capture_close(fd);
	return false;
}

static bool read_usbid(int card, char *id, size_t len)
{
	char path[64];
	snprintf(path, sizeof(path), "/proc/asound/card%d/usbid", card);
	FILE *f = fopen(path, "r");
	if (!f) return false;
	id[0] = 0;
	bool ok = fgets(id, (int)len, f) != NULL;
	fclose(f);
	id[strcspn(id, "\r\n")] = 0;
	return ok;
}

// Prefer the last vendor/product pair across ALSA card-number changes.
// If it cannot be opened, try other USB devices. The ID is not a serial number.
static bool capture_open(struct capture *c, char *last_usbid, size_t idlen)
{
	for (int pass = last_usbid[0] ? 0 : 1; pass < 2; pass++)
	{
		for (int card = 0; card < 8; card++)
		{
			char id[32];
			if (!read_usbid(card, id, sizeof(id))) continue;

			bool same = last_usbid[0] && !strcmp(id, last_usbid);
			if (pass == 0 && !same) continue;
			if (pass == 1 && same) continue;

			char path[64], settled_id[32];
			snprintf(path, sizeof(path), "/dev/snd/pcmC%dD0c", card);
			if (access(path, F_OK)) continue;
			slog("Sampler: card%d (%s) settling before open\n", card, id);
			// Give this candidate time to settle, then check that its card slot
			// still identifies the same model. This does not guarantee readiness.
			for (int i = 0; i < 15 && sampler_wanted; i++) usleep(100000);
			if (!sampler_wanted) return false;
			if (!read_usbid(card, settled_id, sizeof(settled_id)) || strcmp(id, settled_id)) continue;

			if (capture_try_card(card, c))
			{
				snprintf(last_usbid, idlen, "%s", id);
				return true;
			}
		}
	}

	return false;
}

static int16_t sample_s16(const uint8_t *p, int bytes)
{
	// Keep the most significant 16 bits of each supported little-endian format.
	switch (bytes)
	{
	case 2: return (int16_t)(p[0] | (p[1] << 8));
	case 3: return (int16_t)(p[1] | (p[2] << 8));
	default: return (int16_t)(p[2] | (p[3] << 8));
	}
}

// Debug gain override, clamped to 1-64; applied after input detection.
static int read_gain(void)
{
	int gain = 1;
	FILE *f = fopen("/media/fat/config/sampler_gain.txt", "r");
	if (f)
	{
		if (fscanf(f, "%d", &gain) != 1 || gain < 1) gain = 1;
		if (gain > 64) gain = 64;
		fclose(f);
	}
	return gain;
}

// Published settings and status; access across threads is atomic.
// Config encoding is defined in minimig_sampler.h.
static volatile int osd_input;     // main loop -> capture worker
static volatile int file_input;    // capture worker -> OSD, file fallback
static volatile uint32_t sampler_state; // capture worker -> OSD, see state_pack()

enum { SRC_DEFAULT, SRC_OSD, SRC_FILE };

// Zero or nonnumeric/out-of-range input selects Auto. Positive values are
// 1-based ALSA channel numbers; this fallback never overrides an explicit OSD choice.
static int read_channel_file(void)
{
	int ch = 0;
	FILE *f = fopen("/media/fat/config/sampler_channel.txt", "r");
	if (f)
	{
		if (fscanf(f, "%d", &ch) != 1 || ch < 0 || ch > SMP_MAX_INPUT) ch = 0;
		fclose(f);
	}
	return ch;
}

// Explicit Auto must override a manual file fallback, not be treated as unset.
static int resolve_sel(int code, int file_ch, int *src)
{
	if (code == 1) { *src = SRC_OSD; return 0; }
	if (code >= 2 && code <= SMP_MAX_INPUT + 1) { *src = SRC_OSD; return code - 1; }
	if (file_ch > 0) { *src = SRC_FILE; return file_ch; }
	*src = SRC_DEFAULT;
	return 0;
}

// One atomic snapshot: bit 0 = device present, bits 1-6 = hardware channels,
// bits 7-11 = input in use (1-based, zero when none).
static uint32_t state_pack(bool device, int channels, int used)
{
	if (channels > 63) channels = 63;
	return (device ? 1u : 0u) | ((uint32_t)channels << 1) | ((uint32_t)used << 7);
}

static int16_t apply_gain(int16_t v, int gain)
{
	int x = v * gain;
	return (int16_t)(x > 32767 ? 32767 : (x < -32768 ? -32768 : x));
}

// Debug level snapshot, refreshed once per second while capture is running.
static void log_level(int peak, int used, int gain, int sel, int channels)
{
	FILE *f = fopen("/tmp/sampler_level.txt", "w");
	if (!f) return;
	fprintf(f, "peak=%d (of 32767) gain=%d setting=%s used_channel=%d of %d ring_fill=%u..%u dropped=%u\n",
	        peak, gain, sel ? "manual" : "auto", used, channels,
	        ring_fill_min == ~0u ? 0u : ring_fill_min, ring_fill_max, ring_dropped);
	ring_fill_min = ~0u;
	ring_fill_max = 0;
	fclose(f);
}

#define AUTO_MAXCH      SMP_MAX_INPUT   // auto detection only considers inputs 1-16
#define AUTO_THRESHOLD  256             // window peak (of 32767) that counts as signal, about -42 dBFS
#define AUTO_HOLD_WIN   100             // 10 ms windows without signal before another input may take over
#define AUTO_DEC        64
#define AUTO_WINDOWS    5               // look-ahead in 10 ms windows

// Detect on live input; Auto plays from a five-window delay line (~50 ms).
// A newly selected input also gets the pending windows, retaining about 40 ms
// of a quiet attack before it crosses the threshold. Ties prefer the lower
// channel. Keep an active selection; switch only after about one second quiet.
// Manual input bypasses the delay, so Auto/manual changes shift time once.
struct auto_sel
{
	int16_t (*delay)[AUTO_MAXCH];
	int8_t dec[AUTO_DEC];            // zero-based input per 10 ms window; -1 means no selection
	int peak[AUTO_MAXCH];
	uint64_t in_count;
	uint32_t win, dly;
	int cur, quiet;
};

static bool auto_init(struct auto_sel *a, int rate)
{
	memset(a, 0, sizeof(*a));
	a->win = rate / 100;
	a->dly = a->win * AUTO_WINDOWS;
	a->delay = (int16_t (*)[AUTO_MAXCH])calloc(a->dly, sizeof(*a->delay));
	if (!a->delay) return false;
	a->cur = -1;
	memset(a->dec, 0xff, sizeof(a->dec));
	return true;
}

// out receives the delayed frame; the return value selects its input.
// Keep filling the delay in manual mode too, so Auto can resume without a warm-up.
// force pins the detector to a zero-based manual input; -1 allows detection.
static int auto_step(struct auto_sel *a, const int16_t *f, int channels, int16_t *out, int force)
{
	uint32_t pos = (uint32_t)(a->in_count % a->dly);

	memcpy(out, a->delay[pos], sizeof(a->delay[pos]));
	memset(a->delay[pos], 0, sizeof(a->delay[pos]));
	for (int ch = 0; ch < channels; ch++)
	{
		a->delay[pos][ch] = f[ch];
		int v = abs(f[ch]);
		if (v > a->peak[ch]) a->peak[ch] = v;
	}

	uint64_t n = a->in_count++;

	if (force >= 0)
	{
		a->cur = force;
		a->quiet = 0;
	}

	if (((n + 1) % a->win) == 0)
	{
		uint64_t w = n / a->win;

		if (force < 0)
		{
			int best = 0;
			for (int ch = 1; ch < channels; ch++) if (a->peak[ch] > a->peak[best]) best = ch;
			bool active = a->peak[best] >= AUTO_THRESHOLD;
			int old = a->cur;

			if (a->cur < 0 || a->cur >= channels)
			{
				a->cur = active ? best : -1;
				a->quiet = 0;
			}
			else if (a->peak[a->cur] >= AUTO_THRESHOLD)
			{
				a->quiet = 0;
			}
			else
			{
				if (a->quiet < AUTO_HOLD_WIN) a->quiet++;
				if (a->quiet >= AUTO_HOLD_WIN && active && best != a->cur)
				{
					a->cur = best;
					a->quiet = 0;
				}
			}

			// Reassign only pending windows; already emitted audio cannot change.
			if (a->cur != old && a->cur >= 0)
				for (uint64_t k = (w >= AUTO_WINDOWS - 1) ? (w - (AUTO_WINDOWS - 1)) : 0; k <= w; k++)
					a->dec[k % AUTO_DEC] = (int8_t)a->cur;
		}

		a->dec[w % AUTO_DEC] = (int8_t)a->cur;
		memset(a->peak, 0, sizeof(a->peak));
	}

	if (n < a->dly) return -1;
	return a->dec[((n - a->dly) / a->win) % AUTO_DEC];
}

static volatile uint32_t flush_head, flush_gen;

static void capture_inactive()
{
	__atomic_store_n(&sampler_state, state_pack(false, 0, 0), __ATOMIC_RELEASE);
	// Publish the cutoff before its generation. The consumer may see it after
	// capture has restarted, so flushing to the current head would lose new audio.
	__atomic_store_n(&flush_head, __atomic_load_n(&ring_head, __ATOMIC_RELAXED), __ATOMIC_RELEASE);
	__atomic_add_fetch(&flush_gen, 1, __ATOMIC_RELEASE);
}

enum { RUN_LOST, RUN_STOP };

static int capture_run(struct capture *c)
{
	const int frame_bytes = c->channels * c->sample_bytes;
	uint8_t *buf = (uint8_t *)malloc(c->period * frame_bytes);
	if (!buf)
	{
		slog("Sampler: cannot allocate capture buffer\n");
		return RUN_LOST;
	}

	const int nch = (c->channels > AUTO_MAXCH) ? AUTO_MAXCH : c->channels;
	struct auto_sel as;
	if (!auto_init(&as, c->rate))
	{
		slog("Sampler: cannot allocate input delay\n");
		free(buf);
		return RUN_LOST;
	}

	// Phase is measured in input frames and survives ALSA block boundaries.
	// Start at 1.0 to obtain a real previous sample before interpolating.
	const double step = (double)c->rate / SMP_RATE;
	double pos = 1.0;
	int16_t prev_l = 0, prev_r = 0;
	// Windowed-sinc lowpass (cutoff 21 kHz) before decimating captures above
	// 48 kHz. Not used at or below 48 kHz.
	const int fir_n = (step > 1.0) ? (int)(24 * step + 0.5) | 1 : 0;
	float fir_h[SMP_FIR_MAX] = {0}, fir_hist[SMP_FIR_MAX] = {0};
	int fir_pos = 0;
	if (fir_n > 0 && fir_n <= SMP_FIR_MAX)
	{
		const double fc = 21000.0 / c->rate, mid = (fir_n - 1) / 2.0;
		double sum = 0;
		for (int i = 0; i < fir_n; i++)
		{
			double x = i - mid;
			double sinc = (x == 0) ? 2 * fc : sin(2 * M_PI * fc * x) / (M_PI * x);
			fir_h[i] = (float)(sinc * (0.54 - 0.46 * cos(2 * M_PI * i / (fir_n - 1))));
			sum += fir_h[i];
		}
		for (int i = 0; i < fir_n; i++) fir_h[i] = (float)(fir_h[i] / sum);
	}
	int gain = read_gain();
	int file_ch = read_channel_file();
	int sel = 0, src = SRC_DEFAULT;
	int peak = 0, used = 0;
	uint32_t next_slow = now_ms(), state_last = ~0u;
	int recoveries = 0, last_error = 0;
	int reported_missing = 0;
	const char *why = "stopped";

	__atomic_store_n(&file_input, file_ch, __ATOMIC_RELAXED);
	if (capture_ioctl(c->fd, SNDRV_PCM_IOCTL_START, "START") < 0)
	{
		free(as.delay);
		free(buf);
		return RUN_LOST;
	}
	uint32_t last_data = now_ms();

	while (sampler_wanted)
	{
		// Take OSD changes at block boundaries without waiting for the file checks.
		sel = resolve_sel(__atomic_load_n(&osd_input, __ATOMIC_ACQUIRE), file_ch, &src);

		uint32_t t = now_ms();
		// Readiness alone is not progress: EAGAIN and empty reads keep this deadline.
		// It bounds retries, not a kernel ioctl or close() that never returns.
		if (t - last_data >= 3000) { why = "no capture progress for 3 s"; break; }
		if ((int32_t)(t - next_slow) >= 0)
		{
			gain = read_gain();
			file_ch = read_channel_file();
			__atomic_store_n(&file_input, file_ch, __ATOMIC_RELAXED);
			test_tone = access("/media/fat/config/sampler_test", F_OK) == 0;
			test_hz = 0;
			if (test_tone)
			{
				FILE *tf = fopen("/media/fat/config/sampler_test", "r");
				int hz = 0;
				if (tf)
				{
					if (fscanf(tf, "%d", &hz) != 1) hz = 0;
					fclose(tf);
				}
				if ((hz > 0 && hz < 20000) || hz == -1) test_hz = hz; // -1: exact digital silence
			}
			log_level(peak, used, gain, sel, c->channels);
			peak = 0;
			next_slow = t + 1000;
		}

		if (sel > nch)
		{
			if (reported_missing != sel)
			{
				slog("Sampler: input %d not available, the device has %d capture channels (silence)\n", sel, c->channels);
				reported_missing = sel;
			}
		}
		else reported_missing = 0;

		uint32_t st = state_pack(true, c->channels, used);
		if (st != state_last)
		{
			__atomic_store_n(&sampler_state, st, __ATOMIC_RELEASE);
			state_last = st;
		}

		// Short polls keep the worker responsive to settings and the progress deadline.
		struct pollfd pfd = { c->fd, POLLIN, 0 };
		uint32_t remaining = 3000 - (t - last_data);
		int pr = poll(&pfd, 1, remaining < 500 ? (int)remaining : 500);
		if (pr < 0)
		{
			if (errno == EINTR) continue;
			last_error = errno;
			why = "poll error";
			break;
		}
		if (pr == 0) continue;
		if (pfd.revents & POLLNVAL) { why = "poll invalid"; last_error = EBADF; break; }
		if (pfd.revents & POLLHUP) { why = "device disconnected"; last_error = ENODEV; break; }
		// POLLERR also reports XRUN/suspend; a nonblocking read identifies which.
		if (!(pfd.revents & (POLLIN | POLLERR))) continue;

		struct snd_xferi x;
		x.buf = buf;
		x.frames = c->period;
		x.result = 0;

		int result = ioctl(c->fd, SNDRV_PCM_IOCTL_READI_FRAMES, &x);
		int error = result < 0 ? errno : (x.result < 0 ? (int)-x.result : 0);
		if (error)
		{
			if (error == EINTR) continue;
			if (error == EAGAIN) { usleep(1000); continue; }
			if (error == EPIPE || error == ESTRPIPE)
			{
				if (++recoveries > 20) { why = "too many xruns"; last_error = error; break; }
				if (error == ESTRPIPE)
				{
					if (capture_ioctl(c->fd, SNDRV_PCM_IOCTL_RESUME, "RESUME") >= 0) continue;
					if (errno == EAGAIN) { usleep(10000); continue; }
					if (errno != ENOSYS) { why = "resume failed"; last_error = errno; break; }
				}
				if (capture_ioctl(c->fd, SNDRV_PCM_IOCTL_PREPARE, "PREPARE") < 0)
				{
					why = "prepare failed"; last_error = errno; break;
				}
				if (capture_ioctl(c->fd, SNDRV_PCM_IOCTL_START, "START") < 0)
				{
					why = "start failed"; last_error = errno; break;
				}
				continue;
			}
			last_error = error;
			why = (error == ENODEV || error == ENXIO) ? "device removed" : "read error";
			break;
		}
		if (!x.result) { usleep(1000); continue; }
		if (x.result > (long)c->period) { why = "invalid frame count"; last_error = EIO; break; }
		last_data = now_ms();
		recoveries = 0;

		for (long i = 0; i < x.result; i++)
		{
			const uint8_t *s = buf + i * frame_bytes;
			int16_t in[AUTO_MAXCH], dl[AUTO_MAXCH];
			for (int ch = 0; ch < nch; ch++) in[ch] = sample_s16(s + ch * c->sample_bytes, c->sample_bytes);

			int a_ch = auto_step(&as, in, nch, dl, (sel >= 1 && sel <= nch) ? sel - 1 : -1);
			int16_t m;
			if (sel)
			{
				// Manual input has no look-ahead latency. An unavailable channel
				// stays selected and produces silence, never another input.
				m = (sel <= nch) ? in[sel - 1] : 0;
				used = (sel <= nch) ? sel : 0;
			}
			else
			{
				m = (a_ch >= 0) ? dl[a_ch] : 0;
				used = a_ch + 1;
			}

			if (test_tone)
			{
				// Debug source replacement; do not let it influence Auto detection.
				static int tri_pos;
				static double sine_phase;
				int v;
				if (test_hz < 0)
				{
					v = 0;
				}
				else if (test_hz)
				{
					sine_phase += 2 * M_PI * test_hz / c->rate;
					if (sine_phase > 2 * M_PI) sine_phase -= 2 * M_PI;
					v = (int)(26000 * sin(sine_phase));
				}
				else
				{
					tri_pos = (tri_pos + 1) % 240;
					v = (tri_pos < 120) ? tri_pos * 400 - 24000 : 24000 - (tri_pos - 120) * 400;
				}
				m = (int16_t)v;
			}
			if (abs(m) > peak) peak = abs(m);
			int16_t l = (gain > 1) ? apply_gain(m, gain) : m;
			if (fir_n > 0 && fir_n <= SMP_FIR_MAX)
			{
				fir_hist[fir_pos] = l;
				float acc = 0;
				for (int i = 0, k = fir_pos; i < fir_n; i++)
				{
					acc += fir_h[i] * fir_hist[k];
					k = (k == 0) ? fir_n - 1 : k - 1;
				}
				fir_pos = (fir_pos + 1) % fir_n;
				l = (int16_t)(acc > 32767.f ? 32767 : acc < -32768.f ? -32768 : acc);
			}
			int16_t r = l;

			if (c->rate == SMP_RATE)
			{
				ring_put(l, r);
				continue;
			}

			while (pos < 1.0)
			{
				ring_put((int16_t)(prev_l + (l - prev_l) * pos), (int16_t)(prev_r + (r - prev_r) * pos));
				pos += step;
			}
			pos -= 1.0;
			prev_l = l;
			prev_r = r;
		}
	}

	capture_inactive();
	slog("Sampler: capture ended: %s, errno=%d (%s)\n",
	     why, last_error, last_error ? strerror(last_error) : "ok");
	free(as.delay);
	free(buf);
	return sampler_wanted ? RUN_LOST : RUN_STOP;
}

// The worker owns the PCM descriptor for its entire lifetime. Reopening creates
// fresh resampler and detector state; the main loop never touches ALSA.
static void *sampler_main(void *)
{
	char last_usbid[32] = "";
	bool waiting_reported = false;

	for (;;)
	{
		if (!sampler_wanted)
		{
			usleep(200000);
			continue;
		}

		struct capture c;
		if (capture_open(&c, last_usbid, sizeof(last_usbid)))
		{
			waiting_reported = false;
			slog("Sampler: device %s connected\n", last_usbid);
			capture_run(&c);
			capture_close(c.fd);
			slog("Sampler: device %s lost\n", last_usbid);
		}
		else if (!waiting_reported)
		{
			slog("Sampler: waiting for a USB audio capture device\n");
			waiting_reported = true;
		}

		__atomic_store_n(&sampler_state, state_pack(false, 0, 0), __ATOMIC_RELEASE);

		for (int i = 0; i < 10 && sampler_wanted; i++) usleep(100000);
	}
	return NULL;
}

// Output format switch for the FPGA. The default is offset binary (silence =
// 0x80), which ProTracker and OctaMED expect. Creating the file
// /media/fat/config/sampler_signed selects signed output (silence = 0x00).
int minimig_sampler_signed_output(void)
{
	return access("/media/fat/config/sampler_signed", F_OK) == 0;
}

int minimig_sampler_dither_mode(void)
{
	int mode = 0;
	FILE *f = fopen("/media/fat/config/sampler_dither", "r");
	if (f)
	{
		int v;
		if (fscanf(f, "%d", &v) == 1 && v >= 0 && v <= 3) mode = v;
		fclose(f);
	}
	return mode;
}

void minimig_sampler_set_dither(int mode)
{
	FILE *f = fopen("/media/fat/config/sampler_dither", "w");
	if (f)
	{
		fprintf(f, "%d\n", mode & 3);
		fclose(f);
	}
}

const char *minimig_sampler_dither_name(int mode)
{
	static const char *const names[4] = { "Off", "Light", "Normal", "Normal+Gate" };
	return names[mode & 3];
}

unsigned int minimig_sampler_hw_bits(void)
{
	return (minimig_sampler_signed_output() ? SAMPLER_SIGNED_BIT : 0) |
		((unsigned int)minimig_sampler_dither_mode() << SAMPLER_DITHER_SHIFT);
}

void minimig_sampler_describe(char *out, size_t len)
{
	int src;
	int code = (int)((minimig_get_extcfg() >> SAMPLER_CFG_SHIFT) & SAMPLER_CFG_MASK);
	int sel = resolve_sel(code, __atomic_load_n(&file_input, __ATOMIC_RELAXED), &src);
	uint32_t st = __atomic_load_n(&sampler_state, __ATOMIC_ACQUIRE);
	bool device = st & 1;
	int channels = (st >> 1) & 63;
	int used = (st >> 7) & 31;

	if (!device) snprintf(out, len, "No device");
	else if (sel && sel > channels) snprintf(out, len, "Input %d n/a", sel);
	else if (sel) snprintf(out, len, src == SRC_FILE ? "Input %d (file)" : "Input %d", sel);
	else if (used > 0) snprintf(out, len, "Auto (%d)", used);
	else snprintf(out, len, "Auto");
}

void minimig_sampler_poll(void)
{
	static uint32_t last_ms;

	__atomic_store_n(&osd_input, (int)((minimig_get_extcfg() >> SAMPLER_CFG_SHIFT) & SAMPLER_CFG_MASK), __ATOMIC_RELEASE);

	if (!thread_started)
	{
		sampler_wanted = 1;
		if (pthread_create(&sampler_thread, NULL, sampler_main, NULL))
		{
			sampler_wanted = 0;
			slog("Sampler: cannot start capture thread\n");
			return;
		}
		thread_started = 1;
		pthread_detach(sampler_thread);
	}

	uint32_t now = now_ms();
	if (now == last_ms) return;
	last_ms = now;

	uint32_t tail = __atomic_load_n(&ring_tail, __ATOMIC_RELAXED);

	// Only the consumer advances tail. A cutoff can already have been consumed;
	// the signed distance avoids moving tail backwards, including across wrap.
	static uint32_t seen_gen;
	uint32_t gen = __atomic_load_n(&flush_gen, __ATOMIC_ACQUIRE);
	if (gen != seen_gen)
	{
		seen_gen = gen;
		uint32_t cutoff = __atomic_load_n(&flush_head, __ATOMIC_ACQUIRE);
		if ((int32_t)(cutoff - tail) > 0)
		{
			tail = cutoff;
			__atomic_store_n(&ring_tail, tail, __ATOMIC_RELEASE);
		}
	}

	uint32_t head = __atomic_load_n(&ring_head, __ATOMIC_ACQUIRE);
	uint32_t avail = head - tail;

	// A backlog (for example while the core restarts) never drains by itself
	// because input and output run at the same rate. Keep about 20 ms.
	if (avail > SMP_TRIM_ABOVE)
	{
		tail = head - SMP_TRIM_KEEP;
		avail = SMP_TRIM_KEEP;
		__atomic_store_n(&ring_tail, tail, __ATOMIC_RELEASE);
	}

	// Debug transport counters for distinguishing capture stalls from FIFO backpressure.
	static uint32_t dbg_next, dbg_pushed, dbg_polls, dbg_req, dbg_status;
	dbg_polls++;
	if ((int32_t)(now - dbg_next) >= 0)
	{
		FILE *f = fopen("/tmp/sampler_push.txt", "w");
		if (f)
		{
			fprintf(f, "polls=%u avail=%u req_seen=%u frames_pushed=%u last_status=0x%04x\n", dbg_polls, avail, dbg_req, dbg_pushed, dbg_status);
			fclose(f);
		}
		dbg_next = now + 1000;

		// Apply changes to sampler_signed / sampler_dither without a restart.
		static int last_hw = -1;
		int hw = (int)minimig_sampler_hw_bits();
		if (hw != last_hw)
		{
			last_hw = hw;
			minimig_set_extcfg(minimig_get_extcfg());
		}
	}

	// Batch at least 1 ms of audio; bit 14 promises room for the entire burst.
	if (avail < 48) return;

	dbg_status = ide_check();
	if (!(dbg_status & SMP_STATUS_REQ)) return;
	dbg_req++;

	uint32_t n = (avail > SMP_BURST) ? SMP_BURST : avail;
	dbg_pushed += n;

	EnableIO();
	spi8(UIO_DMA_WRITE);
	spi32_w(SMP_ADDR);
	for (uint32_t i = 0; i < n; i++)
	{
		uint32_t f = ring[(tail + i) & (SMP_RING - 1)];
		spi_w((uint16_t)(f >> 16));
		spi_w((uint16_t)f);
	}
	DisableIO();

	__atomic_store_n(&ring_tail, tail + n, __ATOMIC_RELEASE);
}
