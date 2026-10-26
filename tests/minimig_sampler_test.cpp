// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 Dawid Janas
//
// Linux regression tests for the sampler's capture and reconnect paths.
// Scripted ALSA results and virtual time exercise the real implementation.
// FPGA calls are stubs. Tests are sequential and do not model hardware or races.
//
// g++ -std=gnu++14 -Wall -Wextra -pthread tests/minimig_sampler_test.cpp -o /tmp/minimig_sampler_test
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <errno.h>
#include <unistd.h>
#include <fcntl.h>
#include <time.h>
#include <pthread.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <stdarg.h>
#include <sound/asound.h>
#include <cassert>
#include <deque>
#include <map>
#include <string>

struct event
{
	short revents;
	int error;
	long frames;
	int elapsed = 100;
	int poll_result = 1;
	bool result_error = false;
};

static uint64_t clock_ms;
static int opens, closes, open_flags, open_error, close_error, hardware_channels;
static int polls, sleeps;
static std::map<unsigned long, int> calls;
static std::map<unsigned long, std::deque<int>> errors;
static std::map<int, std::string> cards;
static std::deque<event> events;
static event current = {};
static bool repeat_event, replace_during_settle;
static FILE *event_log;
static uint16_t fpga_status;
static unsigned int extcfg;

static int fail(int error)
{
	errno = error;
	return -1;
}

static int test_open(const char *path, int flags)
{
	assert(strncmp(path, "/dev/snd/pcmC", 13) == 0);
	opens++;
	open_flags = flags;
	return open_error ? fail(open_error) : 100;
}

static int test_close(int fd)
{
	assert(fd == 100);
	closes++;
	return close_error ? fail(close_error) : 0;
}

static FILE *test_fopen(const char *path, const char *mode)
{
	if (!strcmp(path, "/tmp/sampler_log.txt"))
		return fdopen(dup(fileno(event_log)), mode);
	if (!strncmp(path, "/tmp/", 5)) return tmpfile();
	int card;
	if (sscanf(path, "/proc/asound/card%d/usbid", &card) == 1 && cards.count(card))
	{
		FILE *f = tmpfile();
		assert(f);
		fputs(cards[card].c_str(), f);
		rewind(f);
		return f;
	}
	errno = ENOENT;
	return NULL;
}

static int test_access(const char *path, int)
{
	int card;
	if (sscanf(path, "/dev/snd/pcmC%dD0c", &card) == 1 && cards.count(card)) return 0;
	return fail(ENOENT);
}

static int test_clock_gettime(clockid_t, struct timespec *ts)
{
	ts->tv_sec = clock_ms / 1000;
	ts->tv_nsec = (clock_ms % 1000) * 1000000;
	return 0;
}

static int test_usleep(useconds_t usec)
{
	sleeps++;
	clock_ms += usec / 1000;
	if (replace_during_settle)
	{
		cards[0] = "2222:0002";
		replace_during_settle = false;
	}
	return 0;
}

static int test_poll(struct pollfd *pfd, nfds_t count, int timeout)
{
	assert(count == 1 && pfd->fd == 100);
	assert(timeout > 0 && timeout <= 500);
	assert(++polls < 100);
	if (!events.empty())
	{
		current = events.front();
		events.pop_front();
	}
	else if (!repeat_event) current = {POLLHUP, 0, 0};
	clock_ms += current.poll_result == 0 ? timeout : current.elapsed;
	pfd->revents = current.revents;
	return current.poll_result < 0 ? fail(current.error) : current.poll_result;
}

static int test_ioctl(int fd, unsigned long command, void *arg)
{
	assert(fd == 100);
	calls[command]++;
	if (!errors[command].empty())
	{
		int error = errors[command].front();
		errors[command].pop_front();
		if (error) return fail(error);
	}
	if (command == SNDRV_PCM_IOCTL_HW_REFINE)
	{
		auto *p = static_cast<snd_pcm_hw_params *>(arg);
		int ch = p->intervals[SNDRV_PCM_HW_PARAM_CHANNELS - SNDRV_PCM_HW_PARAM_FIRST_INTERVAL].min;
		if (ch != hardware_channels) return fail(EINVAL);
	}
	if (command == SNDRV_PCM_IOCTL_READI_FRAMES)
	{
		auto *x = static_cast<snd_xferi *>(arg);
		if (current.error)
		{
			if (!current.result_error) return fail(current.error);
			x->result = -current.error;
			return 0;
		}
		x->result = current.frames;
		if (current.frames > 0 && current.frames <= (long)x->frames)
			memset(x->buf, 0, current.frames * hardware_channels * 2);
	}
	return 0;
}

// Include the implementation to test its private state without exposing a test
// API in production. Replace system calls and MiSTer headers before inclusion.
#define SPI_H
#define USER_IO_H
#define IDE_H
#define __MINIMIG_CONFIG_H__
#define UIO_DMA_WRITE 0x61
static unsigned int minimig_get_extcfg() { return extcfg; }
static void minimig_set_extcfg(unsigned int) {}
static uint16_t ide_check() { return fpga_status; }
static void EnableIO() {}
static void DisableIO() {}
static void spi8(uint8_t) {}
static void spi32_w(uint32_t) {}
static void spi_w(uint16_t) {}
#define open test_open
#define close test_close
#define fopen test_fopen
#define access test_access
#define clock_gettime test_clock_gettime
#define usleep test_usleep
#define poll test_poll
#define ioctl test_ioctl
#include "../support/minimig/minimig_sampler.cpp"
#undef open
#undef close
#undef fopen
#undef access
#undef clock_gettime
#undef usleep
#undef poll
#undef ioctl

static void reset()
{
	clock_ms += 10000;
	opens = closes = open_flags = open_error = close_error = polls = sleeps = 0;
	hardware_channels = 6;
	calls.clear();
	errors.clear();
	events.clear();
	cards.clear();
	repeat_event = replace_during_settle = false;
	current = {};
	fpga_status = extcfg = 0;
	// Keep the generation advancing: the consumer's static seen_gen survives tests.
	ring_head = ring_tail = flush_head = 0;
	flush_gen++;
	osd_input = file_input = sampler_state = 0;
	sampler_wanted = thread_started = 1;
	assert(ftruncate(fileno(event_log), 0) == 0);
	rewind(event_log);
}

static bool logged(const char *text)
{
	rewind(event_log);
	char buf[65536] = {};
	size_t len = fread(buf, 1, sizeof(buf) - 1, event_log);
	buf[len] = 0;
	return strstr(buf, text) != NULL;
}

static void run_capture()
{
	capture c = {100, hardware_channels, 48000, SNDRV_PCM_FORMAT_S16_LE, 2, 480};
	assert(capture_run(&c) == RUN_LOST);
	assert(sampler_state == 0);
	assert(calls[SNDRV_PCM_IOCTL_DROP] == 0);
	assert(calls[SNDRV_PCM_IOCTL_HW_FREE] == 0);
}

static void test_open_failures()
{
	capture c;
	reset();
	open_error = EBUSY;
	assert(!capture_try_card(0, &c));
	assert(open_flags & O_NONBLOCK);
	assert(closes == 0 && calls[SNDRV_PCM_IOCTL_HW_REFINE] == 0);

	const unsigned long commands[] = {
		SNDRV_PCM_IOCTL_HW_REFINE, SNDRV_PCM_IOCTL_HW_PARAMS, SNDRV_PCM_IOCTL_PREPARE
	};
	for (unsigned long command : commands)
	{
		for (int error : {ENODEV, ENXIO, EIO, EBUSY})
		{
			reset();
			errors[command].push_back(error);
			assert(!capture_try_card(0, &c));
			assert(calls[command] == 1 && closes == 1);
			assert(calls[SNDRV_PCM_IOCTL_HW_PARAMS] <= 1);
			assert(calls[SNDRV_PCM_IOCTL_PREPARE] <= 1);
		}
	}

	reset();
	assert(capture_try_card(0, &c));
	assert(c.channels == 6 && c.rate == 48000);
	assert(calls[SNDRV_PCM_IOCTL_HW_REFINE] == 26 * 18 + 1);
	assert(calls[SNDRV_PCM_IOCTL_HW_PARAMS] == 1);
	assert(calls[SNDRV_PCM_IOCTL_PREPARE] == 1);
	assert((open_flags & (O_NONBLOCK | O_CLOEXEC)) == (O_NONBLOCK | O_CLOEXEC));
	capture_close(c.fd);
	assert(closes == 1);
}

static void test_disconnect_and_transient_errors()
{
	for (short flags : {POLLHUP, POLLNVAL, POLLHUP | POLLIN})
	{
		reset();
		events.push_back({flags, 0, 0});
		run_capture();
		assert(calls[SNDRV_PCM_IOCTL_READI_FRAMES] == 0);
	}
	for (bool result_error : {false, true})
	{
		reset();
		events.push_back({POLLERR, ENODEV, 0, 100, 1, result_error});
		run_capture();
		assert(calls[SNDRV_PCM_IOCTL_READI_FRAMES] == 1);
		assert(calls[SNDRV_PCM_IOCTL_PREPARE] == 0);
		assert(logged("device removed"));
	}
	reset();
	events.push_back({POLLIN, EAGAIN, 0});
	events.push_back({POLLIN, EINTR, 0});
	events.push_back({POLLIN, 0, 12});
	run_capture();
	assert(ring_head == 12 && polls == 4);
	assert(calls[SNDRV_PCM_IOCTL_PREPARE] == 0);

	reset();
	events.push_back({0, EINTR, 0, 100, -1});
	events.push_back({POLLIN, 0, 48});
	run_capture();
	assert(ring_head == 48 && polls == 3);

	reset();
	errors[SNDRV_PCM_IOCTL_START].push_back(ENODEV);
	run_capture();
	assert(polls == 0 && calls[SNDRV_PCM_IOCTL_READI_FRAMES] == 0);
	assert(logged("START end result=-1"));
}

static void test_progress_deadline()
{
	// All three cases lack frames, even though two keep reporting readiness.
	for (event e : {event{0, 0, 0, 0, 0}, event{POLLIN, EAGAIN, 0}, event{POLLIN, 0, 0}})
	{
		reset();
		current = e;
		repeat_event = true;
		uint64_t begin = clock_ms;
		run_capture();
		assert(clock_ms - begin >= 3000 && clock_ms - begin < 3200);
		assert(logged("no capture progress"));
	}
	reset();
	events.push_back({POLLIN, 0, 481});
	run_capture();
	assert(ring_head == 0 && logged("invalid frame count"));
}

static void test_recovery()
{
	reset();
	events.push_back({POLLERR, EPIPE, 0});
	events.push_back({POLLIN, 0, 48});
	run_capture();
	assert(calls[SNDRV_PCM_IOCTL_PREPARE] == 1);
	assert(calls[SNDRV_PCM_IOCTL_START] == 2 && ring_head == 48);

	reset();
	events.push_back({POLLERR, EPIPE, 0});
	errors[SNDRV_PCM_IOCTL_PREPARE].push_back(ENODEV);
	run_capture();
	assert(calls[SNDRV_PCM_IOCTL_START] == 1);
	assert(logged("prepare failed"));

	reset();
	events.push_back({POLLERR, ESTRPIPE, 0});
	run_capture();
	assert(calls[SNDRV_PCM_IOCTL_RESUME] == 1);
	assert(calls[SNDRV_PCM_IOCTL_PREPARE] == 0);

	reset();
	events.push_back({POLLERR, ESTRPIPE, 0});
	errors[SNDRV_PCM_IOCTL_RESUME].push_back(ENOSYS);
	run_capture();
	assert(calls[SNDRV_PCM_IOCTL_PREPARE] == 1);

	reset();
	events.push_back({POLLERR, ESTRPIPE, 0});
	errors[SNDRV_PCM_IOCTL_RESUME].push_back(ENODEV);
	run_capture();
	assert(calls[SNDRV_PCM_IOCTL_PREPARE] == 0 && logged("resume failed"));

	reset();
	events.push_back({POLLERR, ESTRPIPE, 0});
	events.push_back({POLLERR, ESTRPIPE, 0});
	events.push_back({POLLIN, 0, 48});
	errors[SNDRV_PCM_IOCTL_RESUME].push_back(EAGAIN);
	run_capture();
	assert(calls[SNDRV_PCM_IOCTL_RESUME] == 2);
	assert(calls[SNDRV_PCM_IOCTL_PREPARE] == 0 && ring_head == 48);

	reset();
	current = {POLLERR, EPIPE, 0, 0};
	repeat_event = true;
	run_capture();
	assert(calls[SNDRV_PCM_IOCTL_PREPARE] == 20);
	assert(logged("too many xruns"));
}

static void test_replug_and_flush()
{
	reset();
	char last_id[32] = "";
	capture c;
	assert(!capture_open(&c, last_id, sizeof(last_id)));
	assert(opens == 0 && sleeps == 0);
	for (int card = 0; card < 3; card++)
	{
		cards.clear();
		cards[card] = card == 1 ? "2222:0002" : "1111:0001";
		hardware_channels = card == 1 ? 12 : 6;
		uint64_t begin = clock_ms;
		assert(capture_open(&c, last_id, sizeof(last_id)));
		assert(clock_ms - begin == 1500);
		assert(c.channels == hardware_channels);
		events.push_back({POLLIN, 0, 48});
		assert(capture_run(&c) == RUN_LOST);
		assert(sampler_state == 0);
		capture_close(c.fd);
	}
	assert(opens == 3 && closes == 3);

	reset();
	cards[0] = "1111:0001";
	last_id[0] = 0;
	replace_during_settle = true;
	assert(!capture_open(&c, last_id, sizeof(last_id)));
	assert(opens == 0);

	reset();
	for (int i = 0; i < 100; i++) ring_put(0, 0);
	// A delayed consumer must drop old frames without dropping the new stream.
	capture_inactive();
	for (int i = 0; i < 60; i++) ring_put(123, 123);
	minimig_sampler_poll();
	assert(ring_tail == 100 && ring_head == 160);
	fpga_status = SMP_STATUS_REQ;
	clock_ms++;
	minimig_sampler_poll();
	assert(ring_tail == 160);

	reset();
	ring_head = ring_tail = UINT32_MAX - 19;
	for (int i = 0; i < 40; i++) ring_put(0, 0);
	capture_inactive();
	for (int i = 0; i < 60; i++) ring_put(123, 123);
	minimig_sampler_poll();
	assert(ring_tail == 20 && ring_head == 80);
}

int main()
{
	event_log = tmpfile();
	assert(event_log);
	test_open_failures();
	test_disconnect_and_transient_errors();
	test_progress_deadline();
	test_recovery();
	test_replug_and_flush();
	reset();
	close_error = EINTR;
	capture_close(100);
	assert(closes == 1 && logged("close end result=-1"));
	fclose(event_log);
	puts("PASS: sampler hotplug, progress deadlines, recovery and ring cutoff");
}
