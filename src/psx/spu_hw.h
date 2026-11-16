/* Hooks for hardware voices (built with DD2_SPU_HW; the Dreamcast's AICA in platform/kos_spu.c).
 *
 * libspu keeps every voice's state exactly (envelope, position, loops: what the game reads back) but
 * makes no sound itself: each voice plays on a hardware channel from its sample decoded once to 16-bit
 * PCM, its pitch and volume updated every vblank. The CD audio still goes out through the platform's
 * audio stream. */
#ifndef PSX_SPU_HW_H
#define PSX_SPU_HW_H

#include <stdint.h>

/* Copies a decoded sample to sound memory: a handle, or -1 when there is no room. */
int spu_hw_upload(const int16_t *pcm, int samples);
void spu_hw_free(int handle);

/* Starts voice `voice` (0-23) on sample `handle`, `length` samples, looping back to `loop_start` at
 * the end or playing once when it is -1; `freq` in Hz, `vol` 0-255 (linear), `pan` 0 left, 128
 * centre, 255 right. */
void spu_hw_play(int voice, int handle, int length, int loop_start, int freq, int vol, int pan);
void spu_hw_update(int voice, int freq, int vol, int pan);
void spu_hw_stop(int voice);

#endif
