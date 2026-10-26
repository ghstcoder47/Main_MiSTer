# Main_MiSTer Main Binary and Wiki Repo

This repo serves as the home for the MiSTer Main binaries and the Wiki.

For the purposes of getting google to crawl the wiki, here's a link to the (not for humans) [crawlable wiki](https://github-wiki-see.page/m/MiSTer-devel/Wiki_MiSTer/wiki)

If you're a human looking for the wiki, that's [here](https://github.com/MiSTer-devel/Wiki_MiSTer/wiki)

To compile this application, read more about that [here](https://mister-devel.github.io/MkDocs_MiSTer/developer/mistercompile/#general-prerequisites-for-arm-cross-compiling)

This version has been modified to add support for MiSTer Floppy, SCP and IPF support - see https://mister.robsmithdev.co.uk

## USB sampler for the Minimig (Amiga) core

This fork adds a USB audio sampler to the Minimig core. A class-compliant USB
audio interface plugged into the MiSTer works as a parallel-port sampler for
trackers such as OctaMED or ProTracker. This is the ARM side (capture with
ALSA, level handling, OSD); the FPGA side is in the matching Minimig core
fork. Install both together, see the release package for `install.sh` and a
user guide.

- OSD: **Audio & Video > USB Capture** (Auto or input 1-16) and
  **Capture Dither** (Off, Light, Normal, Normal+Gate; default Off).
- Optional files in `/media/fat/config/`: `sampler_gain.txt`,
  `sampler_channel.txt`, `sampler_dither`, `sampler_signed`.
- Diagnostics: `/tmp/sampler_level.txt`, `/tmp/sampler_log.txt`. The capture
  thread opens devices nonblocking, reopens after disconnect or when no data
  arrives for three seconds, and logs open/close events.
- After installing a new core file, reload the core (or reboot). Restarting
  only the main binary does not reload the FPGA.

Hardware-independent regression tests (Linux):

```sh
g++ -std=gnu++14 -Wall -Wextra -pthread tests/minimig_sampler_test.cpp -o /tmp/minimig_sampler_test
/tmp/minimig_sampler_test
```
