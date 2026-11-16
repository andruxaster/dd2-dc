#include "psx/libspu.h"

#include "platform/platform.h"
#include "psx/libcd.h"
#include "psx/profile.h"
#ifdef DD2_SPU_HW
#include "psx/spu_hw.h"
#endif

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

/* An SPU emulation behind the Psy-Q calls, following psx-spx ("Sound Processing Unit"): voices
 * decode SPU-ADPCM from SPU RAM at their pitch with the 4-point Gaussian interpolation, through
 * the ADSR envelope and their volumes, mixed with the CD input at 44.1 kHz. Reverb, noise and
 * pitch modulation are not emulated yet (the game does not turn them on so far). */

#define SPU_RATE 44100
#define MAX_VBLANK_FRAMES 800
#define VOICES 24
#define BLOCK_SAMPLES 28
#define BLOCK_BYTES 16

/* The first 0x1010 bytes of SPU RAM are reserved (capture buffers and the SPU's own data). */
#define SPU_RAM_RESERVED 0x1010
#define DUMMY_BLOCK 0x1000 /* where _spu_init puts a silent looping block for idle voices */

/* ADPCM header flags */
#define FLAG_LOOP_END 1
#define FLAG_LOOP_REPEAT 2
#define FLAG_LOOP_START 4

enum { ENV_OFF, ENV_ATTACK, ENV_DECAY, ENV_SUSTAIN, ENV_RELEASE };

typedef struct {
    /* registers */
    uint16_t volume[2];  /* VOLL/VOLR, direct mode: volume / 2 */
    uint16_t pitch;
    uint16_t start;      /* in 8-byte units */
    uint16_t adsr1, adsr2;
    uint16_t repeat;     /* LSAX */
    /* state */
    uint32_t address;    /* current block, in 8-byte units */
    uint32_t counter;    /* pitch counter: bit 12 up the sample in the block, bits 4..11 interpolation */
    int16_t samples[3 + BLOCK_SAMPLES]; /* the previous block's last 3, then this block's */
    int16_t history[2];  /* ADPCM filter: last and second to last decoded */
    uint8_t flags;       /* the current block's header flags */
    bool decoded;
    int phase;           /* ENV_* */
    int32_t level;       /* ADSR volume, 0..7FFFh */
    uint32_t env_counter;
    bool keyed;          /* keyed on since the last spu_service (DD2_SPU_HW) */
} Voice;

extern const int16_t spu_gauss[512];

static u_char ram[SPU_RAM_SIZE];
static u_long malloc_next;
static u_long transfer_addr;
static SpuCommonAttr common;
static Voice voices[VOICES];
static uint32_t endx;
static int64_t vblanks_mixed;
#ifdef DD2_SPU_HW
static void hw_forget(uint32_t first, uint32_t end);
#endif

const u_char *spu_ram(void)
{
    return ram;
}

/* As _spu_init: voices silent at pitch 3FFFh on the dummy block, envelopes zero. */
void SpuInit(void)
{
    static const u_char dummy[BLOCK_BYTES] = {7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7}; /* 0x800789A0 */
    memset(ram, 0, sizeof(ram));
    memcpy(ram + DUMMY_BLOCK, dummy, sizeof(dummy));
    memset(voices, 0, sizeof(voices));
    for (int v = 0; v < VOICES; v++) {
        voices[v].pitch = 0x3fff;
        voices[v].start = DUMMY_BLOCK / 8;
    }
    endx = 0;
    malloc_next = SPU_RAM_RESERVED;
}

/* Psy-Q keeps a block table in `top`; the game never frees SPU memory, so a bump allocator does. */
/* Releases the SPU's transfer callback and event; the port has neither. */
void SpuQuit(void)
{
}

long SpuInitMalloc(long num, char *top)
{
    (void)top;
    malloc_next = SPU_RAM_RESERVED;
    return num;
}

long SpuMalloc(long size)
{
    size = (size + 7) & ~7L;
    if (malloc_next + (u_long)size > SPU_RAM_SIZE)
        return -1;
    long addr = (long)malloc_next;
    malloc_next += (u_long)size;
    return addr;
}

void SpuSetCommonAttr(SpuCommonAttr *attr)
{
    if (attr->mask & SPU_COMMON_MVOLL)
        common.mvol.left = attr->mvol.left;
    if (attr->mask & SPU_COMMON_MVOLR)
        common.mvol.right = attr->mvol.right;
    if (attr->mask & SPU_COMMON_CDVOLL)
        common.cd.volume.left = attr->cd.volume.left;
    if (attr->mask & SPU_COMMON_CDVOLR)
        common.cd.volume.right = attr->cd.volume.right;
}

long SpuSetReverb(long on_off)
{
    return on_off;
}

long SpuSetTransferMode(long mode)
{
    return mode;
}

u_long SpuSetTransferStartAddr(u_long addr)
{
    transfer_addr = addr;
    return addr;
}

u_long SpuWrite(u_char *addr, u_long size)
{
    if (transfer_addr >= SPU_RAM_SIZE)
        return 0;
    if (size > SPU_RAM_SIZE - transfer_addr)
        size = SPU_RAM_SIZE - transfer_addr;
    memcpy(ram + transfer_addr, addr, size);
#ifdef DD2_SPU_HW
    hw_forget((uint32_t)transfer_addr, (uint32_t)(transfer_addr + size));
#endif
    return size;
}

long SpuIsTransferCompleted(long flag)
{
    (void)flag;
    return 1;
}

/* Only the volume modes the game uses are emulated: direct volume (mode 0). */
static uint16_t volume_register(short volume, short mode)
{
    static bool warned;
    if (mode != 0 && !warned) {
        fprintf(stderr, "libspu: volume mode %d is played as direct volume\n", mode);
        warned = true;
    }
    return (uint16_t)(volume & 0x7fff);
}

/* Writes the registers the mask selects into every voice the attribute names. */
void SpuSetVoiceAttr(SpuVoiceAttr *attr)
{
    for (int i = 0; i < VOICES; i++) {
        if (!(attr->voice & (1u << i)))
            continue;
        Voice *v = &voices[i];
        if (attr->mask & SPU_VOICE_VOLL)
            v->volume[0] = volume_register(attr->volume.left, attr->volmode.left);
        if (attr->mask & SPU_VOICE_VOLR)
            v->volume[1] = volume_register(attr->volume.right, attr->volmode.right);
        if (attr->mask & SPU_VOICE_PITCH)
            v->pitch = attr->pitch;
        if (attr->mask & SPU_VOICE_WDSA)
            v->start = (uint16_t)(attr->addr >> 3);
        if (attr->mask & SPU_VOICE_LSAX)
            v->repeat = (uint16_t)(attr->loop_addr >> 3);
        if (attr->mask & SPU_VOICE_ADSR_ADSR1)
            v->adsr1 = attr->adsr1;
        if (attr->mask & SPU_VOICE_ADSR_ADSR2)
            v->adsr2 = attr->adsr2;
    }
}

/* Key on restarts the sample with the envelope at zero; key off starts the release. */
void SpuSetKey(long on_off, u_long voice_bit)
{
    for (int i = 0; i < VOICES; i++) {
        if (!(voice_bit & (1u << i)))
            continue;
        Voice *v = &voices[i];
        if (on_off == SPU_ON) {
            v->address = v->start;
            v->counter = 0;
            v->decoded = false;
            memset(v->samples, 0, sizeof(v->samples));
            v->history[0] = v->history[1] = 0;
            v->phase = ENV_ATTACK;
            v->level = 0;
            v->env_counter = 0;
            v->keyed = true;
            endx &= ~(1u << i);
        } else if (v->phase != ENV_OFF) {
            v->phase = ENV_RELEASE;
            v->env_counter = 0;
        }
    }
}

static int16_t clamp16(int32_t v)
{
    return (int16_t)(v > 32767 ? 32767 : v < -32768 ? -32768 : v);
}

/* Decodes the block at the voice's address; a Loop Start flag makes it the repeat address. Shift
 * values 13..15 act as 9 and filters past 4 as 4 (as DuckStation; psx-spx leaves them open). */
static void decode_block(Voice *v)
{
    static const int filter_pos[5] = {0, 60, 115, 98, 122};
    static const int filter_neg[5] = {0, 0, -52, -55, -60};
    const u_char *block = &ram[(v->address * 8) & (SPU_RAM_SIZE - 1)];
    int shift = block[0] & 0x0f;
    int filter = (block[0] >> 4) & 7;
    if (shift > 12)
        shift = 9;
    if (filter > 4)
        filter = 4;
    v->flags = block[1];
    if (v->flags & FLAG_LOOP_START)
        v->repeat = (uint16_t)v->address;
    memmove(v->samples, v->samples + BLOCK_SAMPLES, 3 * sizeof(int16_t));
    for (int i = 0; i < BLOCK_SAMPLES; i++) {
        int nibble = (block[2 + i / 2] >> ((i & 1) * 4)) & 0xf;
        int32_t t = (int32_t)((uint32_t)nibble << 28) >> 28;
        int32_t s = t * (1 << (12 - shift)) +
                    (v->history[0] * filter_pos[filter] + v->history[1] * filter_neg[filter] + 32) / 64;
        int16_t sample = clamp16(s);
        v->history[1] = v->history[0];
        v->history[0] = sample;
        v->samples[3 + i] = sample;
    }
    v->decoded = true;
}

/* One envelope step from psx-spx "Envelope Operation depending on Shift/Step/Mode/Direction":
 * the counter rises by `increment` each tick and the level moves by `step` when its bit 15 sets. */
static void envelope_rate(const Voice *v, bool exponential, bool decrease, int shift, int step_bits,
                          int32_t *step, uint32_t *increment)
{
    int32_t adsr_step = 7 - step_bits;
    if (decrease)
        adsr_step = ~adsr_step;
    adsr_step *= 1 << (shift < 11 ? 11 - shift : 0);
    uint32_t inc = 0x8000u >> (shift > 11 ? shift - 11 : 0);
    if (exponential && !decrease && v->level > 0x6000) {
        if (shift < 10) {
            adsr_step /= 4;
        } else if (shift >= 11) {
            inc /= 4;
        } else {
            adsr_step /= 2;
            inc /= 2;
        }
    } else if (exponential && decrease) {
        adsr_step = adsr_step * v->level / 0x8000;
    }
    if ((step_bits | shift << 2) != 0x7f && inc == 0)
        inc = 1;
    *step = adsr_step;
    *increment = inc;
}

static void envelope_tick(Voice *v, bool exponential, bool decrease, int shift, int step_bits)
{
    int32_t adsr_step;
    uint32_t increment;
    envelope_rate(v, exponential, decrease, shift, step_bits, &adsr_step, &increment);
    v->env_counter += increment;
    if (!(v->env_counter & 0x8000))
        return;
    v->env_counter = 0;
    v->level += adsr_step;
    if (!decrease)
        v->level = v->level > 0x7fff ? 0x7fff : v->level;
    else if (v->level < 0)
        v->level = 0;
}

/* The envelope settings of the voice's phase (false when it has none: off). */
static bool envelope_params(const Voice *v, bool *exponential, bool *decrease, int *shift, int *step_bits)
{
    switch (v->phase) {
    case ENV_ATTACK:
        *exponential = v->adsr1 >> 15, *decrease = false, *shift = (v->adsr1 >> 10) & 0x1f;
        *step_bits = (v->adsr1 >> 8) & 3;
        return true;
    case ENV_DECAY:
        *exponential = true, *decrease = true, *shift = (v->adsr1 >> 4) & 0xf, *step_bits = 0;
        return true;
    case ENV_SUSTAIN:
        *exponential = v->adsr2 >> 15, *decrease = (v->adsr2 >> 14) & 1, *shift = (v->adsr2 >> 8) & 0x1f;
        *step_bits = (v->adsr2 >> 6) & 3;
        return true;
    case ENV_RELEASE:
        *exponential = (v->adsr2 >> 5) & 1, *decrease = true, *shift = v->adsr2 & 0x1f, *step_bits = 0;
        return true;
    default:
        return false;
    }
}

static void envelope(Voice *v)
{
    bool exponential, decrease;
    int shift, step_bits;
    if (!envelope_params(v, &exponential, &decrease, &shift, &step_bits))
        return;
    envelope_tick(v, exponential, decrease, shift, step_bits);
    switch (v->phase) {
    case ENV_ATTACK:
        if (v->level >= 0x7fff) {
            v->phase = ENV_DECAY;
            v->env_counter = 0;
        }
        break;
    case ENV_DECAY:
        if (v->level <= ((v->adsr1 & 0xf) + 1) * 0x800) {
            v->phase = ENV_SUSTAIN;
            v->env_counter = 0;
        }
        break;
    case ENV_RELEASE:
        if (v->level == 0)
            v->phase = ENV_OFF;
        break;
    default:
        break;
    }
}

/* One 44.1 kHz sample of a voice, after its envelope, into the left and right sums. */
static void voice_sample(int index, int32_t *left, int32_t *right)
{
    Voice *v = &voices[index];
    if (!v->decoded)
        decode_block(v);
    int s = (int)(v->counter >> 12);
    int g = (int)(v->counter >> 4) & 0xff;
    int32_t out = (spu_gauss[0xff - g] * v->samples[s] >> 15) + (spu_gauss[0x1ff - g] * v->samples[s + 1] >> 15) +
                  (spu_gauss[0x100 + g] * v->samples[s + 2] >> 15) + (spu_gauss[g] * v->samples[s + 3] >> 15);
    out = out * v->level >> 15;
    *left += out * (int16_t)(v->volume[0] << 1) >> 15;
    *right += out * (int16_t)(v->volume[1] << 1) >> 15;

    envelope(v);
    v->counter += v->pitch > 0x3fff ? 0x4000 : v->pitch;
    if ((v->counter >> 12) >= BLOCK_SAMPLES) {
        v->counter -= BLOCK_SAMPLES << 12;
        if (v->flags & FLAG_LOOP_END) {
            endx |= 1u << index;
            v->address = v->repeat;
            if (!(v->flags & FLAG_LOOP_REPEAT)) {
                v->phase = ENV_RELEASE;
                v->level = 0;
            }
        } else {
            v->address += BLOCK_BYTES / 8;
        }
        v->decoded = false;
    }
}

#ifdef DD2_SPU_HW
/* Ticks the envelope up to n times, as n calls of envelope() would; returns how many ran (fewer when the
 * voice went off). Ticks that only raise the counter are taken together: between the level changes the
 * rate stays the same and so does every phase check. */
static int envelope_advance(Voice *v, int n)
{
    int done = 0;
    while (done < n) {
        int phase = v->phase;
        int32_t level = v->level;
        envelope(v);
        done++;
        if (v->phase == ENV_OFF)
            return done;
        if (v->phase != phase || v->level != level || v->env_counter == 0)
            continue; /* a step or a new phase: the next tick is worked out afresh */
        bool exponential, decrease;
        int shift, step_bits;
        int32_t step;
        uint32_t increment;
        if (!envelope_params(v, &exponential, &decrease, &shift, &step_bits))
            return done;
        envelope_rate(v, exponential, decrease, shift, step_bits, &step, &increment);
        if (increment == 0) {
            done = n; /* never steps */
            break;
        }
        /* Ticks before the one whose increment sets bit 15. */
        uint32_t quiet = (0x8000u - v->env_counter + increment - 1) / increment - 1;
        if (quiet > (uint32_t)(n - done))
            quiet = (uint32_t)(n - done);
        v->env_counter += quiet * increment;
        done += (int)quiet;
    }
    return done;
}

/* The block the voice enters: its flags (a Loop Start flag makes it the repeat address), as decode_block
 * sets them, without decoding. */
static void enter_block(Voice *v)
{
    const u_char *block = &ram[(v->address * 8) & (SPU_RAM_SIZE - 1)];
    v->flags = block[1];
    if (v->flags & FLAG_LOOP_START)
        v->repeat = (uint16_t)v->address;
    v->decoded = true;
}

/* n samples of a voice's state as voice_sample moves it (envelope, position, loop flags, ENDX), without
 * making its sound, by whole stretches between block ends. */
static void voice_advance(int index, int n)
{
    Voice *v = &voices[index];
    while (n > 0 && v->phase != ENV_OFF) {
        if (!v->decoded)
            enter_block(v);
        uint32_t pitch = v->pitch > 0x3fff ? 0x4000 : v->pitch;
        /* Samples up to and including the one whose step leaves the block. */
        uint32_t to_end = pitch ? ((BLOCK_SAMPLES << 12) - v->counter + pitch - 1) / pitch : 0xffffffffu;
        int run = to_end < (uint32_t)n ? (int)to_end : n;
        int ticked = envelope_advance(v, run);
        v->counter += (uint32_t)ticked * pitch;
        n -= ticked;
        if ((v->counter >> 12) >= BLOCK_SAMPLES) {
            v->counter -= BLOCK_SAMPLES << 12;
            if (v->flags & FLAG_LOOP_END) {
                endx |= 1u << index;
                v->address = v->repeat;
                if (!(v->flags & FLAG_LOOP_REPEAT)) {
                    v->phase = ENV_RELEASE;
                    v->level = 0;
                }
            } else {
                v->address += BLOCK_BYTES / 8;
            }
            v->decoded = false;
        }
        if (ticked < run)
            break; /* went off */
    }
}
#endif

void SpuGetVoiceEnvelope(int voice, short *envx)
{
    *envx = (short)voices[voice].level;
}

void spu_mix(short *stereo, int frames)
{
    cd_audio_read(stereo, frames);
    for (int i = 0; i < frames; i++) {
        int32_t l = stereo[i * 2] * common.cd.volume.left >> 15;
        int32_t r = stereo[i * 2 + 1] * common.cd.volume.right >> 15;
        for (int v = 0; v < VOICES; v++)
            if (voices[v].phase != ENV_OFF)
                voice_sample(v, &l, &r);
        l = clamp16(l);
        r = clamp16(r);
        stereo[i * 2] = clamp16(l * common.mvol.left >> 14);
        stereo[i * 2 + 1] = clamp16(r * common.mvol.right >> 14);
    }
}


#ifdef DD2_SPU_HW
/* ---- hardware voices (spu_hw.h) ----
 *
 * A sample is what a voice plays from its start address to the block with Loop End, decoded as
 * decode_block does; it loops (Loop Repeat in that block) from the last Loop Start block on the way, or
 * the repeat address the voice had. Longer than a hardware channel takes (65535 samples), every 2 (or 4)
 * are averaged into one and it plays at half (a quarter) the rate.
 *
 * PORT: the sound follows the emulated voices at the vblank: volume (envelope included), pitch and pan
 * change 60 times a second instead of every sample; interpolation is the hardware's, not the Gaussian;
 * a loop restarts the ADPCM filter's history from the decoded sample, not from the loop's own past. */

#define HW_SAMPLES 64
#define HW_MAX_LENGTH 65535
#define HW_MAX_BLOCKS (SPU_RAM_SIZE / BLOCK_BYTES)

typedef struct {
    bool used;
    uint32_t start, end;  /* the ADPCM in SPU RAM, bytes */
    int loop_block;       /* the block it loops back to, -1 for none */
    int handle, length, loop_start, shift;
    unsigned last_use;
} HwSample;

static HwSample hw_samples[HW_SAMPLES];
static struct {
    int sample; /* hw_samples index while playing, -1 when not */
    int freq, vol, pan;
} hw_voices[VOICES];
static unsigned hw_clock;
static int16_t hw_pcm[HW_MAX_LENGTH];

static void hw_init_voices(void)
{
    static bool done;
    if (done)
        return;
    done = true;
    for (int i = 0; i < VOICES; i++)
        hw_voices[i].sample = -1;
}

/* Stops what plays a sample and frees it. */
static void hw_drop(int index)
{
    for (int i = 0; i < VOICES; i++)
        if (hw_voices[i].sample == index) {
            spu_hw_stop(i);
            hw_voices[i].sample = -1;
        }
    spu_hw_free(hw_samples[index].handle);
    hw_samples[index].used = false;
}

/* SPU RAM first..end-1 was written: the samples from it are decoded again when next played. */
static void hw_forget(uint32_t first, uint32_t end)
{
    for (int i = 0; i < HW_SAMPLES; i++)
        if (hw_samples[i].used && hw_samples[i].start < end && first < hw_samples[i].end)
            hw_drop(i);
}

/* Frees the least recently started sample no voice plays; false when there is none. */
static bool hw_evict(void)
{
    int pick = -1;
    for (int i = 0; i < HW_SAMPLES; i++) {
        if (!hw_samples[i].used)
            continue;
        bool playing = false;
        for (int v = 0; v < VOICES && !playing; v++)
            playing = hw_voices[v].sample == i;
        if (!playing && (pick < 0 || hw_samples[i].last_use < hw_samples[pick].last_use))
            pick = i;
    }
    if (pick < 0)
        return false;
    hw_drop(pick);
    return true;
}

/* The sample voice v plays from its start address, decoded and uploaded if needed; -1 if it can't be. */
static int hw_sample_for(const Voice *v)
{
    uint32_t start = (uint32_t)v->start * 8;
    /* Where it ends and loops: the walk the voice would make. */
    int blocks = 0, loop_block = -1;
    uint32_t repeat = (uint32_t)v->repeat * 8;
    bool loops = false;
    for (uint32_t a = start;; a = (a + BLOCK_BYTES) & (SPU_RAM_SIZE - 1)) {
        uint8_t flags = ram[a + 1];
        if (flags & FLAG_LOOP_START)
            repeat = a;
        blocks++;
        if (flags & FLAG_LOOP_END) {
            loops = (flags & FLAG_LOOP_REPEAT) != 0;
            break;
        }
        if (blocks >= HW_MAX_BLOCKS)
            break;
    }
    uint32_t end = start + (uint32_t)blocks * BLOCK_BYTES;
    if (loops)
        loop_block = repeat >= start && repeat < end ? (int)((repeat - start) / BLOCK_BYTES) : 0;
    for (int i = 0; i < HW_SAMPLES; i++)
        if (hw_samples[i].used && hw_samples[i].start == start && hw_samples[i].loop_block == loop_block) {
            hw_samples[i].last_use = hw_clock;
            return i;
        }

    int shift = 0;
    while (((long)blocks * BLOCK_SAMPLES >> shift) > HW_MAX_LENGTH)
        shift++;
    int per = 1 << shift, length = 0, sum = 0, summed = 0;
    Voice d;
    memset(&d, 0, sizeof(d));
    for (int b = 0; b < blocks; b++) {
        d.address = (start / 8 + (uint32_t)b * 2) & (SPU_RAM_SIZE / 8 - 1);
        decode_block(&d);
        for (int k = 0; k < BLOCK_SAMPLES; k++) {
            sum += d.samples[3 + k];
            if (++summed == per) {
                if (length < HW_MAX_LENGTH)
                    hw_pcm[length++] = (int16_t)(sum / per);
                sum = summed = 0;
            }
        }
    }
    if (length == 0)
        return -1;
    int handle;
    while ((handle = spu_hw_upload(hw_pcm, length)) < 0)
        if (!hw_evict())
            return -1;
    int pick = -1;
    for (int i = 0; i < HW_SAMPLES && pick < 0; i++)
        if (!hw_samples[i].used)
            pick = i;
    if (pick < 0) {
        if (!hw_evict()) {
            spu_hw_free(handle);
            return -1;
        }
        for (int i = 0; i < HW_SAMPLES && pick < 0; i++)
            if (!hw_samples[i].used)
                pick = i;
    }
    HwSample *h = &hw_samples[pick];
    h->used = true;
    h->start = start;
    h->end = end;
    h->loop_block = loop_block;
    h->handle = handle;
    h->length = length;
    h->loop_start = loop_block < 0 ? -1 : loop_block * BLOCK_SAMPLES >> shift;
    h->shift = shift;
    h->last_use = hw_clock;
    return pick;
}

/* A voice's output gain on one side, 0..7FFFh: envelope, voice volume (direct mode, either sign) and
 * the main volume, as spu_mix applies them. */
static int32_t hw_gain(const Voice *v, int side)
{
    int32_t vol = (int16_t)(v->volume[side] << 1);
    if (vol < 0)
        vol = -vol;
    int32_t main = side ? common.mvol.right : common.mvol.left;
    if (main < 0)
        main = -main;
    int32_t g = (v->level * vol >> 15) * main >> 14;
    return g > 0x7fff ? 0x7fff : g;
}

/* Gains to the hardware's volume (linear 0-255) and pan (3 dB steps away from the centre). */
static void hw_volume_pan(int32_t left, int32_t right, int *vol, int *pan)
{
    int32_t loud = left > right ? left : right, quiet = left > right ? right : left;
    *vol = (int)((loud * 255 + 0x4000) / 0x7fff);
    int steps = 0;
    if (quiet <= 0) {
        steps = loud > 0 ? 15 : 0;
    } else {
        /* 3 dB = 2^(1/2): count how many halvings of the power separate the sides. */
        int64_t q = (int64_t)quiet * quiet, l = (int64_t)loud * loud;
        while (steps < 15 && q * 2 <= l) {
            q *= 2;
            steps++;
        }
        if (steps < 15 && q * 2 - l < l - q) /* nearer the next step */
            steps++;
    }
    if (steps == 0)
        *pan = 128;
    else if (left > right)
        *pan = 0x7f - steps * 8;
    else
        *pan = 0x80 + steps * 8;
}

/* A vblank of the voices: their exact state, then what the hardware channels play. */
static void hw_service(int frames)
{
    hw_init_voices();
    hw_clock++;
    for (int i = 0; i < VOICES; i++) {
        Voice *v = &voices[i];
        bool keyed = v->keyed;
        v->keyed = false;
        voice_advance(i, frames);
        if (v->phase == ENV_OFF) {
            if (hw_voices[i].sample >= 0) {
                spu_hw_stop(i);
                hw_voices[i].sample = -1;
            }
            continue;
        }
        int vol, pan;
        hw_volume_pan(hw_gain(v, 0), hw_gain(v, 1), &vol, &pan);
        int index = hw_voices[i].sample;
        uint32_t pitch = v->pitch > 0x3fff ? 0x4000 : v->pitch;
        if (pitch == 0)
            vol = 0; /* a stopped voice; the hardware has no rate 0 */
        int shift = index >= 0 ? hw_samples[index].shift : 0;
        if (keyed) {
            index = hw_sample_for(v);
            hw_voices[i].sample = index;
            if (index < 0) {
                spu_hw_stop(i);
                continue;
            }
            const HwSample *h = &hw_samples[index];
            int freq = (int)((SPU_RATE * pitch / 4096) >> h->shift);
            spu_hw_play(i, h->handle, h->length, h->loop_start, freq, vol, pan);
            hw_voices[i].freq = freq;
            hw_voices[i].vol = vol;
            hw_voices[i].pan = pan;
        } else if (index >= 0) {
            int freq = (int)((SPU_RATE * pitch / 4096) >> shift);
            if (freq != hw_voices[i].freq || vol != hw_voices[i].vol || pan != hw_voices[i].pan) {
                spu_hw_update(i, freq, vol, pan);
                hw_voices[i].freq = freq;
                hw_voices[i].vol = vol;
                hw_voices[i].pan = pan;
            }
        }
    }
}

/* The CD input alone, at the CD and main volumes. */
static void hw_cd_mix(short *stereo, int frames)
{
    cd_audio_read(stereo, frames);
    for (int i = 0; i < frames; i++) {
        int32_t l = stereo[i * 2] * common.cd.volume.left >> 15;
        int32_t r = stereo[i * 2 + 1] * common.cd.volume.right >> 15;
        stereo[i * 2] = clamp16(l * common.mvol.left >> 14);
        stereo[i * 2 + 1] = clamp16(r * common.mvol.right >> 14);
    }
}
#endif

/* 44100 Hz against the 59.94 Hz vblank: 735 or 736 frames per vblank. */
void spu_service(void)
{
    int64_t start = vblanks_mixed * SPU_RATE * 1001 / 60000;
    vblanks_mixed++;
    int frames = (int)(vblanks_mixed * SPU_RATE * 1001 / 60000 - start);
    short out[MAX_VBLANK_FRAMES * 2];
    PROFILE_START();
#ifdef DD2_SPU_HW
    hw_service(frames);
    hw_cd_mix(out, frames);
#else
    spu_mix(out, frames);
#endif
    PROFILE_END(PROFILE_SPU);
    platform_audio_push(out, frames);
}
