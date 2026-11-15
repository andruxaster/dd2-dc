/* The platform layer for the Sega Dreamcast on KallistiOS: the same interface as platform_sdl.c.
 *
 * Video: the PSX display area is converted into one 1024x512 16-bit PVR texture (PSX VRAM's size, so every
 * display mode fits) and drawn as a single quad stretched to 640x480; the PVR does the scaling.
 * Sound: the SPU's 44.1 kHz stereo output goes through a ring into a KOS sound stream.
 * Input: the first maple controller, mapped to PSX button positions (A = Cross, B = Circle, X = Square,
 * Y = Triangle). */
#include "platform/platform.h"

#include <dc/biosfont.h>
#include <kos.h>
#include <stdio.h>
#include <string.h>

KOS_INIT_FLAGS(INIT_DEFAULT);

/* PSX digital pad bits. */
enum {
    PSX_SELECT = 1 << 0, PSX_L3 = 1 << 1, PSX_R3 = 1 << 2, PSX_START = 1 << 3,
    PSX_UP = 1 << 4, PSX_RIGHT = 1 << 5, PSX_DOWN = 1 << 6, PSX_LEFT = 1 << 7,
    PSX_L2 = 1 << 8, PSX_R2 = 1 << 9, PSX_L1 = 1 << 10, PSX_R1 = 1 << 11,
    PSX_TRIANGLE = 1 << 12, PSX_CIRCLE = 1 << 13, PSX_CROSS = 1 << 14, PSX_SQUARE = 1 << 15,
};

/* The software path's picture (movies, and everything when the PVR renderer is off). */
#define TEX_W 512
#define TEX_H 512
#define TRIGGER_HALF 128
#define STICK_DEAD 64

static bool test_run;
static pvr_ptr_t texture;
static bool frame_rgb565; /* the last frame came from the 24-bit mode */
static int frame_w, frame_h;

static const struct {
    uint32_t cont;
    uint16_t bit;
} button_map[] = {
    {CONT_DPAD_UP, PSX_UP}, {CONT_DPAD_DOWN, PSX_DOWN}, {CONT_DPAD_LEFT, PSX_LEFT}, {CONT_DPAD_RIGHT, PSX_RIGHT},
    {CONT_START, PSX_START}, {CONT_A, PSX_CROSS}, {CONT_B, PSX_CIRCLE}, {CONT_X, PSX_SQUARE},
    {CONT_Y, PSX_TRIANGLE}, {CONT_Z, PSX_SELECT}, {CONT_D, PSX_SELECT}, {CONT_C, PSX_R1},
};

/* ---- sound ---- */

/* About 8 vblanks of sound; more than that queued means the game ran ahead, so it is dropped. */
#define AUDIO_RING_FRAMES 8192
#define AUDIO_MAX_QUEUED (8 * 736)
#define STREAM_BUFFER 8192

static snd_stream_hnd_t stream = SND_STREAM_INVALID;
static int16_t ring[AUDIO_RING_FRAMES * 2];
static unsigned ring_read, ring_write; /* in stereo frames, wrapping */
static int16_t stream_out[STREAM_BUFFER / 2] __attribute__((aligned(32)));

/* Called from snd_stream_poll on the main thread, so the ring needs no lock. `bytes` is what the
 * stream wants; any shortfall is silence. */
static void *stream_callback(snd_stream_hnd_t hnd, int bytes, int *got)
{
    (void)hnd;
    int frames = bytes / 4;
    if (frames > (int)(sizeof(stream_out) / 4))
        frames = (int)(sizeof(stream_out) / 4);
    unsigned queued = ring_write - ring_read;
    for (int i = 0; i < frames; i++) {
        if ((unsigned)i < queued) {
            unsigned at = (ring_read + (unsigned)i) % AUDIO_RING_FRAMES;
            stream_out[i * 2] = ring[at * 2];
            stream_out[i * 2 + 1] = ring[at * 2 + 1];
        } else {
            stream_out[i * 2] = stream_out[i * 2 + 1] = 0;
        }
    }
    ring_read += (unsigned)frames < queued ? (unsigned)frames : queued;
    *got = frames * 4;
    return stream_out;
}

void platform_audio_push(const int16_t *stereo, int frames)
{
    if (stream == SND_STREAM_INVALID || ring_write - ring_read > AUDIO_MAX_QUEUED)
        return;
    for (int i = 0; i < frames; i++) {
        unsigned at = ring_write % AUDIO_RING_FRAMES;
        ring[at * 2] = stereo[i * 2];
        ring[at * 2 + 1] = stereo[i * 2 + 1];
        ring_write++;
    }
}

bool platform_audio_record(const char *path)
{
    (void)path;
    return false;
}

/* ---- video ---- */

bool platform_init(const char *title, bool test)
{
    (void)title;
    test_run = test;
    vid_set_mode(DM_640x480, PM_RGB565);
    /* Opaque, translucent and punch-through lists (kos_gpu.c draws transparent texels punch-through). */
    pvr_init_params_t params = {
        .opb_sizes = {PVR_BINSIZE_16, PVR_BINSIZE_0, PVR_BINSIZE_16, PVR_BINSIZE_0, PVR_BINSIZE_16},
        .vertex_buf_size = 768 * 1024,
        .opb_overflow_count = 1,
    };
    if (pvr_init(&params) < 0)
        return false;
    pvr_set_bg_color(0, 0, 0);
    texture = pvr_mem_malloc(TEX_W * TEX_H * 2);
    if (!texture)
        return false;
    memset(texture, 0, TEX_W * TEX_H * 2);
    if (!test_run && snd_stream_init() == 0) {
        stream = snd_stream_alloc(stream_callback, STREAM_BUFFER);
        if (stream != SND_STREAM_INVALID)
            snd_stream_start(stream, 44100, 1);
    }
    return true;
}

void platform_shutdown(void)
{
    if (stream != SND_STREAM_INVALID) {
        snd_stream_stop(stream);
        snd_stream_destroy(stream);
        snd_stream_shutdown();
        stream = SND_STREAM_INVALID;
    }
}

/* PSX 15-bit pixels are mask:B:G:R; the PVR's ARGB1555 is A:R:G:B, so red and blue swap. Two pixels
 * at a time when the source is word aligned (the SH4 faults on unaligned loads). */
static void convert_row_15(uint16_t *dst, const uint16_t *src, int w)
{
    if (((uintptr_t)src & 3) == 0) {
        const uint32_t *s = (const uint32_t *)src;
        uint32_t *d = (uint32_t *)dst;
        for (int i = 0; i < (w + 1) / 2; i++) {
            uint32_t p = s[i];
            d[i] = 0x80008000u | (p & 0x03e003e0u) | (p & 0x001f001fu) << 10 | (p >> 10 & 0x001f001fu);
        }
        return;
    }
    for (int i = 0; i < w; i++) {
        uint16_t p = src[i];
        dst[i] = (uint16_t)(0x8000u | (p & 0x03e0u) | (p & 0x001fu) << 10 | (p >> 10 & 0x001fu));
    }
}

/* ---- profiling (DD2_PROFILE): a line of figures over the picture, also printed to the console ---- */

#ifdef DD2_PROFILE
#ifdef DD2_HW_RENDER
void kos_gpu_stats(int *converted, int *failed);
#endif
#define PROFILE_PERIOD_NS 2000000000ull

static uint64_t profile_ns[PROFILE_PARTS], profile_since;
static int profile_frames;
static char profile_line[64];

void platform_profile_add(int part, uint64_t ns)
{
    profile_ns[part] += ns;
}

/* Game frames per second and the share of time in the GPU, the SPU and showing the picture. */
void platform_profile_frame(void)
{
    uint64_t now = timer_ns_gettime64();
    if (profile_since == 0)
        profile_since = now;
    profile_frames++;
    uint64_t elapsed = now - profile_since;
    if (elapsed < PROFILE_PERIOD_NS)
        return;
    int pct[PROFILE_PARTS];
    for (int i = 0; i < PROFILE_PARTS; i++) {
        pct[i] = (int)(profile_ns[i] * 100 / elapsed);
        profile_ns[i] = 0;
    }
    int fps10 = (int)((uint64_t)profile_frames * 10000000000ull / elapsed);
    int converted = 0, failed = 0;
#ifdef DD2_HW_RENDER
    kos_gpu_stats(&converted, &failed);
#endif
    snprintf(profile_line, sizeof(profile_line), "%d.%dfps G%d S%d P%d T%d X%d", fps10 / 10, fps10 % 10,
             pct[PROFILE_GPU], pct[PROFILE_SPU], pct[PROFILE_PRESENT], converted, failed);
    printf("dd2 profile: %s (%% of time: G gpu, S spu, P present; T textures made, X primitives dropped)\n",
           profile_line);
    profile_frames = 0;
    profile_since = now;
}

static void draw_profile_line(void)
{
    if (profile_line[0] && frame_h >= BFONT_HEIGHT)
        bfont_draw_str_ex((uint16_t *)texture + (frame_h - BFONT_HEIGHT) * TEX_W, TEX_W, 0xffff,
                          frame_rgb565 ? 0x0000 : 0x8000, 16, true, profile_line);
}
#else
static void draw_profile_line(void)
{
}
#endif

/* Draws the texture's top-left w x h texels over the whole screen. Skips the frame when the PVR is
 * still busy with the last one: the game's pace comes from the emulated vblank, never from here. */
static void draw_frame(void)
{
    if (pvr_check_ready() < 0)
        return;
    pvr_scene_begin();
    pvr_list_begin(PVR_LIST_OP_POLY);
    if (frame_w > 0 && frame_h > 0) {
        pvr_poly_cxt_t cxt;
        pvr_poly_hdr_t hdr;
        pvr_poly_cxt_txr(&cxt, PVR_LIST_OP_POLY,
                         (frame_rgb565 ? PVR_TXRFMT_RGB565 : PVR_TXRFMT_ARGB1555) | PVR_TXRFMT_NONTWIDDLED,
                         TEX_W, TEX_H, texture, PVR_FILTER_NONE);
        pvr_poly_compile(&hdr, &cxt);
        pvr_prim(&hdr, sizeof(hdr));

        float u = (float)frame_w / TEX_W, v = (float)frame_h / TEX_H;
        pvr_vertex_t vert = {0};
        vert.argb = 0xffffffff;
        vert.z = 1.0f;
        const float corners[4][4] = {{0, 0, 0, 0}, {640, 0, 1, 0}, {0, 480, 0, 1}, {640, 480, 1, 1}};
        for (int i = 0; i < 4; i++) {
            vert.flags = i == 3 ? PVR_CMD_VERTEX_EOL : PVR_CMD_VERTEX;
            vert.x = corners[i][0];
            vert.y = corners[i][1];
            vert.u = corners[i][2] * u;
            vert.v = corners[i][3] * v;
            pvr_prim(&vert, sizeof(vert));
        }
    }
    pvr_list_finish();
    pvr_scene_finish();
}

void platform_present(const uint16_t *pixels, int w, int h, int pitch)
{
    if (!pixels) {
        frame_w = frame_h = 0;
        draw_frame();
        return;
    }
    if (w > TEX_W)
        w = TEX_W;
    if (h > TEX_H)
        h = TEX_H;
    uint16_t *dst = texture;
    for (int y = 0; y < h; y++)
        convert_row_15(dst + y * TEX_W, pixels + y * pitch, w);
    frame_w = w;
    frame_h = h;
    frame_rgb565 = false;
    draw_profile_line();
    draw_frame();
}

/* The 24-bit mode (movies): R, G, B bytes to RGB565. */
void platform_present_rgb24(const uint8_t *pixels, int w, int h, int pitch)
{
    if (w > TEX_W)
        w = TEX_W;
    if (h > TEX_H)
        h = TEX_H;
    uint16_t *dst = texture;
    for (int y = 0; y < h; y++) {
        const uint8_t *s = pixels + y * pitch;
        uint16_t *d = dst + y * TEX_W;
        for (int x = 0; x < w; x++, s += 3)
            d[x] = (uint16_t)((s[0] & 0xf8) << 8 | (s[1] & 0xfc) << 3 | s[2] >> 3);
    }
    frame_w = w;
    frame_h = h;
    frame_rgb565 = true;
    draw_profile_line();
    draw_frame();
}

/* Screenshots are a desktop test tool. */
bool platform_save_bmp(const char *path, const uint16_t *pixels, int w, int h, int pitch)
{
    (void)path, (void)pixels, (void)w, (void)h, (void)pitch;
    return false;
}

bool platform_save_bmp_rgb24(const char *path, const uint8_t *pixels, int w, int h, int pitch)
{
    (void)path, (void)pixels, (void)w, (void)h, (void)pitch;
    return false;
}

/* ---- input ---- */

static cont_state_t *pad_state(void)
{
    maple_device_t *dev = maple_enum_type(0, MAPLE_FUNC_CONTROLLER);
    return dev ? (cont_state_t *)maple_dev_status(dev) : NULL;
}

/* Keeps the sound stream fed; A+B+X+Y+Start (the Dreamcast's usual reset) quits. */
bool platform_pump_events(void)
{
    if (stream != SND_STREAM_INVALID)
        snd_stream_poll(stream);
    cont_state_t *st = pad_state();
    return st && (st->buttons & CONT_RESET_BUTTONS) == CONT_RESET_BUTTONS;
}

/* The standard pad has no shoulder buttons or Select: the left trigger is L1 (fast steer in the default
 * layout), the right trigger R2 (view), both triggers together Select (Start + Select abandons a race).
 * The analog stick also works as the D-pad. */
uint16_t platform_pad_buttons(void)
{
    if (test_run)
        return 0;
    cont_state_t *st = pad_state();
    if (!st)
        return 0;
    uint16_t buttons = 0;
    for (size_t i = 0; i < sizeof(button_map) / sizeof(button_map[0]); i++)
        if (st->buttons & button_map[i].cont)
            buttons |= button_map[i].bit;
    bool lt = st->ltrig > TRIGGER_HALF, rt = st->rtrig > TRIGGER_HALF;
    if (lt && rt)
        buttons |= PSX_SELECT;
    else if (lt)
        buttons |= PSX_L1;
    else if (rt)
        buttons |= PSX_R2;
    if (st->joyx < -STICK_DEAD)
        buttons |= PSX_LEFT;
    if (st->joyx > STICK_DEAD)
        buttons |= PSX_RIGHT;
    if (st->joyy < -STICK_DEAD)
        buttons |= PSX_UP;
    if (st->joyy > STICK_DEAD)
        buttons |= PSX_DOWN;
    return buttons;
}

/* NeGcon: twist from the stick, I from the right trigger (accelerate), II from the left. */
void platform_pad_analog(PlatformAnalog *a)
{
    a->twist = 0x80;
    a->i = a->ii = a->l = 0;
    cont_state_t *st = test_run ? NULL : pad_state();
    if (!st)
        return;
    a->twist = (uint8_t)(st->joyx + 128);
    a->i = (uint8_t)st->rtrig;
    a->ii = (uint8_t)st->ltrig;
    a->l = (st->buttons & CONT_Z) ? 0xff : 0;
}

/* ---- time and storage ---- */

uint64_t platform_ticks_ns(void)
{
    return timer_ns_gettime64();
}

void platform_sleep_until_ns(uint64_t ticks_ns)
{
    for (;;) {
        uint64_t now = timer_ns_gettime64();
        if (now >= ticks_ns)
            return;
        uint64_t left = ticks_ns - now;
        if (left > 2000000)
            thd_sleep((unsigned)(left / 1000000) - 1);
        else
            thd_pass();
    }
}

/* PORT: a 128 KB PSX card image is larger than a whole VMU, so cards live in memory only for now. */
bool platform_card_path(int slot, char *path, int size)
{
    (void)slot, (void)path, (void)size;
    return false;
}
