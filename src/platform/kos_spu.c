/* The AICA voices behind psx/spu_hw.h (Dreamcast, DD2_SPU_HW): a sound channel for each of the 24 PSX
 * voices, samples in sound RAM, driven through the KallistiOS sound driver's channel commands. */
#include "psx/spu_hw.h"

#include <dc/sound/aica_comm.h>
#include <dc/sound/sfxmgr.h>
#include <dc/sound/sound.h>
#include <dc/spu.h>
#include <kos.h>
#include <stdio.h>
#include <string.h>

#define VOICES 24
#define MAX_HANDLES 64

static int channel[VOICES];
static bool have_channels, no_channels;
static uint32_t handles[MAX_HANDLES]; /* sound RAM addresses, 0 when free */
static int16_t upload_buf[2048] __attribute__((aligned(32)));

static bool channels(void)
{
    if (have_channels)
        return true;
    if (no_channels)
        return false;
    for (int i = 0; i < VOICES; i++) {
        channel[i] = snd_sfx_chn_alloc();
        if (channel[i] < 0) {
            printf("kos_spu: only %d sound channels\n", i);
            while (i-- > 0)
                snd_sfx_chn_free(channel[i]);
            no_channels = true;
            return false;
        }
    }
    have_channels = true;
    return true;
}

int spu_hw_upload(const int16_t *pcm, int samples)
{
    int h = 0;
    while (h < MAX_HANDLES && handles[h])
        h++;
    if (h == MAX_HANDLES)
        return -1;
    size_t bytes = ((size_t)samples * 2 + 31) & ~(size_t)31;
    uint32_t addr = snd_mem_malloc(bytes);
    if (!addr)
        return -1;
    /* Through an aligned buffer: spu_memload copies in 32-bit words. */
    for (size_t done = 0; done < bytes; done += sizeof(upload_buf)) {
        size_t n = bytes - done < sizeof(upload_buf) ? bytes - done : sizeof(upload_buf);
        size_t have = (size_t)samples * 2 > done ? (size_t)samples * 2 - done : 0;
        memset(upload_buf, 0, n);
        memcpy(upload_buf, (const uint8_t *)pcm + done, have < n ? have : n);
        spu_memload(addr + done, upload_buf, n);
    }
    handles[h] = addr;
    return h;
}

void spu_hw_free(int handle)
{
    if (handle < 0 || handle >= MAX_HANDLES || !handles[handle])
        return;
    snd_mem_free(handles[handle]);
    handles[handle] = 0;
}

static void channel_command(int voice, uint32_t command, int handle, int length, int loop_start, int freq, int vol, int pan)
{
    AICA_CMDSTR_CHANNEL(tmp, cmd, chan);
    memset(chan, 0, sizeof(*chan));
    cmd->cmd = AICA_CMD_CHAN;
    cmd->timestamp = 0;
    cmd->size = AICA_CMDSTR_CHANNEL_SIZE;
    cmd->cmd_id = (uint32_t)channel[voice];
    chan->cmd = command;
    if (handle >= 0) {
        chan->base = handles[handle];
        chan->type = AICA_SM_16BIT;
        chan->length = (uint32_t)length;
        chan->loop = loop_start >= 0;
        chan->loopstart = loop_start >= 0 ? (uint32_t)loop_start : 0;
        chan->loopend = (uint32_t)length;
    }
    chan->freq = (uint32_t)(freq > 0 ? freq : 1);
    chan->vol = (uint32_t)vol;
    chan->pan = (uint32_t)pan;
    snd_sh4_to_aica(tmp, cmd->size);
}

void spu_hw_play(int voice, int handle, int length, int loop_start, int freq, int vol, int pan)
{
    if (channels() && handle >= 0 && handles[handle])
        channel_command(voice, AICA_CH_CMD_START, handle, length, loop_start, freq, vol, pan);
}

void spu_hw_update(int voice, int freq, int vol, int pan)
{
    if (channels())
        channel_command(voice, AICA_CH_CMD_UPDATE | AICA_CH_UPDATE_SET_FREQ | AICA_CH_UPDATE_SET_VOL | AICA_CH_UPDATE_SET_PAN,
             -1, 0, -1, freq, vol, pan);
}

void spu_hw_stop(int voice)
{
    if (channels())
        channel_command(voice, AICA_CH_CMD_STOP, -1, 0, -1, 44100, 0, 128);
}
