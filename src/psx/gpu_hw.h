/* Hooks for a hardware renderer (built with DD2_HW_RENDER; the Dreamcast's PVR in platform/kos_gpu.c).
 *
 * Primitives drawn into a screen buffer (a drawing area that gets displayed) go to the renderer instead of
 * the software rasterizer; everything the CPU writes (LoadImage, ClearImage, MoveImage, fills) still goes to
 * the VRAM copy, which the renderer reads textures from and shows under the primitives. Drawing anywhere
 * else (into textures) stays in software. Coordinates are absolute VRAM ones (drawing offset applied). */
#ifndef PSX_GPU_HW_H
#define PSX_GPU_HW_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    int x, y;
    uint8_t r, g, b;
    uint8_t u, v;
} HwVertex;

typedef struct {
    bool textured, raw, semi;
    unsigned texpage;            /* E1 bits 0-8: page, semi-transparency mode, depth */
    unsigned clut;
    unsigned tw;                 /* texture window: mask x, mask y, offset x, offset y (5 bits each) */
} HwState;

/* True while the current drawing area is recorded by the renderer. */
bool gpu_hw_drawing(void);

/* A triangle (3) or a quad (4, PSX vertex order: triangles 0-1-2 and 1-2-3). */
void gpu_hw_polygon(const HwState *s, const HwVertex *v, int count);

/* A rectangle at the absolute VRAM position; texture corner (u, v). */
void gpu_hw_rect(const HwState *s, int x, int y, int w, int h, uint8_t r, uint8_t g, uint8_t b, int u, int v);

/* A one-pixel line. */
void gpu_hw_line(const HwState *s, const HwVertex *a, const HwVertex *b);

/* The CPU (or the software rasterizer) wrote this VRAM rectangle: cached textures from it are stale.
 * `cpu` is false for software-rasterized drawing, which never shows as a screen's background. */
void gpu_hw_vram_written(int x, int y, int w, int h, bool cpu);

/* MoveImage from (sx, sy) to (dx, dy): a whole recorded screen copied to another screen copies its record. */
void gpu_hw_move(int sx, int sy, int w, int h, int dx, int dy);

/* PutDrawEnv with this clipping area (inclusive); PutDispEnv with this display area. */
void gpu_hw_draw_env(int x1, int y1, int x2, int y2);
void gpu_hw_disp_env(int x, int y, int w, int h, bool rgb24);

/* Shows the display area if the renderer has it; false leaves it to the software path. */
bool gpu_hw_present(int x, int y, int w, int h);

#endif
