// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 Dawid Janas
//
// Main-loop and OSD interface to USB capture. The worker stays private to
// minimig_sampler.cpp; polling is the only entry point that transfers to the FPGA.

#ifndef MINIMIG_SAMPLER_H
#define MINIMIG_SAMPLER_H

#include <stddef.h>

// Call from the main loop only: publishes OSD settings and feeds the FPGA FIFO.
void minimig_sampler_poll(void);

// extcfg bits 27-31 are saved in the existing Minimig config layout:
// 0 = unset, 1 = explicit Auto, 2..17 = inputs 1..16, 18..31 = invalid.
// Unset/invalid falls back to sampler_channel.txt, then Auto. Explicit Auto
// overrides the file. These bits map to FPGA status[63:59], unused by the core.
#define SAMPLER_CFG_SHIFT 27
#define SAMPLER_CFG_MASK  0x1f

// 1 = FPGA outputs signed 8 bit (silence 0x00), 0 = offset binary (silence 0x80).
// Sent to the core as extcfg bit 24 (status[56]) whenever extcfg is applied.
#define SAMPLER_SIGNED_BIT (1u << 24)
int minimig_sampler_signed_output(void);

// Dither mode from /media/fat/config/sampler_dither (0 off, 1 low, 2 normal
// TPDF = default, 3 gated TPDF). Sent as extcfg bits 25-26 (status[58:57]).
#define SAMPLER_DITHER_SHIFT 25
#define SAMPLER_DITHER_MASK  (3u << SAMPLER_DITHER_SHIFT)
int minimig_sampler_dither_mode(void);
void minimig_sampler_set_dither(int mode);        // persists to sampler_dither
const char *minimig_sampler_dither_name(int mode); // 15 chars max

// Flag bits (signed + dither) that are not stored in the saved config.
#define SAMPLER_HW_MASK (SAMPLER_SIGNED_BIT | SAMPLER_DITHER_MASK)
unsigned int minimig_sampler_hw_bits(void);

// Main-thread formatter. Combines its saved selection with an atomic worker
// snapshot; a selected input is not necessarily present on the current device.
void minimig_sampler_describe(char *out, size_t len);

#endif
