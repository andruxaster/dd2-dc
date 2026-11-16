/* The PVR renderer behind psx/gpu_hw.h (Dreamcast, DD2_HW_RENDER).
 *
 * Each screen buffer the game draws into has a record: the frame's primitives as ready PVR polygons. A
 * record is shown when the game displays its buffer. Draw order (the PSX ordering table) becomes depth:
 * every primitive sits a little nearer than the one before, so the opaque, punch-through and translucent
 * lists combine in the original order. Under the primitives goes the buffer's content in the VRAM copy
 * (what LoadImage, ClearImage, MoveImage and fills put there), as a solid colour when it is uniform.
 *
 * Textures: the part of a texture page a primitive uses (its UVs rounded out to an aligned power-of-two
 * block, 8x8 up to the whole 256x256 page), decoded through its CLUT, becomes an ARGB1555 PVR texture
 * (colour 0 transparent, drawn punch-through). A cache keeps the blocks until the VRAM under them is
 * written; a block is freed only when no recorded frame uses it.
 *
 * A semi-transparent textured primitive is drawn twice, as on the PSX only texels with bit 15 (STP) set
 * blend: its STP-clear texels punch-through, its STP-set texels translucent (two variants of the block).
 *
 * PORT: approximations: mode 2 (B-F) is drawn as B*(1-F); colour modulation stops at 2x
 * brightness of the texel; no dithering, mask bit or drawing-area clipping; reading back a screen buffer
 * (StoreImage, MoveImage of part of it) sees the VRAM copy, which holds no primitives. */
#include "psx/gpu_hw.h"

#include <kos.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* From libgpu.c (psx/libgpu.h can't be included next to KOS headers: its u_int clashes with newlib's). */
uint16_t *gpu_vram(void);

#define VRAM_W 1024
#define VRAM_H 512
#define SCREEN_W 640.0f
#define SCREEN_H 480.0f
#define MAX_SLOTS 3
#define MAX_PRIMS 4096
#define MAX_TEXTURES 2048
#define BUCKETS 256
#define TEXTURE_BUDGET (2560u * 1024u)
#define PAGE 256
#define BACKDROP 512
#define Z_STEP (1.0f / 4096.0f)

/* The header and vertices are contiguous, so a primitive goes to the PVR in one store-queue copy; 192
 * bytes keeps every primitive 32-byte aligned. */
typedef struct {
    pvr_poly_hdr_t hdr;
    pvr_vertex_t v[4];
    uint32_t hdr_id; /* which compiled header hdr is: equal ids are equal headers */
    uint8_t list, count;
    int16_t tex;
    uint8_t pad[24];
} Prim;

_Static_assert(sizeof(Prim) == 192, "Prim");

typedef struct {
    bool used, shown, single, pending;
    int x, y, w, h;
    float sx, sy;
    Prim *prims;
    int count, base;
    int mark;                    /* while pending: where the new frame's primitives start */
    unsigned gen, rendered_gen;
    unsigned cpu_gen, backdrop_gen;
    bool backdrop_solid;
    uint16_t backdrop_colour;
    pvr_ptr_t backdrop;
    int backdrop_h;
    unsigned last_use;
    bool full_reported;
} Slot;

typedef struct {
    bool used, stale;
    unsigned page, clut, tw; /* page: E1 bits 0-4 and depth (bits 7-8) */
    int bu, bv, bw, bh;      /* the block of the page, in texels */
    int variant;             /* VARIANT_*: which texels it keeps */
    bool empty;              /* no texel kept */
    pvr_ptr_t ptr;
    int x0, y0, x1, y1;      /* its texels in VRAM, end exclusive */
    int cx0, cy, cx1;        /* the CLUT */
    unsigned last_use;
    int refs;                /* recorded primitives drawing with it */
    int next;                /* the next block in its hash bucket, -1 at the end */
} Texture;

static bool ready;
static Slot slots[MAX_SLOTS];
static Slot *cur;
static Texture textures[MAX_TEXTURES];
static int buckets[BUCKETS]; /* first block of each (page, CLUT, window) hash, -1 when empty */
static int free_list = -1;   /* unused blocks, linked through next */
static size_t texture_bytes;
static int conversions, failures; /* since the last kos_gpu_stats */
static unsigned frame_counter;
static struct {
    int x, y, w, h;
    bool rgb24;
} disp;
static uint16_t convert_buf[PAGE * PAGE] __attribute__((aligned(32)));
static uint16_t row_buf[BACKDROP] __attribute__((aligned(32)));

static void init(void)
{
    ready = true;
    for (int i = 0; i < BUCKETS; i++)
        buckets[i] = -1;
    for (int i = MAX_TEXTURES - 1; i >= 0; i--) {
        textures[i].next = free_list;
        free_list = i;
    }
    for (int i = 0; i < MAX_SLOTS; i++) {
        slots[i].prims = aligned_alloc(32, sizeof(Prim) * MAX_PRIMS);
        if (!slots[i].prims) {
            printf("kos_gpu: no memory for frame records\n");
            ready = false;
            return;
        }
    }
    PVR_SET(PVR_PT_ALPHA_REF, 0x80);
    printf("kos_gpu: %u bytes of PVR memory free, %u for textures\n", (unsigned)pvr_mem_available(),
           TEXTURE_BUDGET);
}

/* Which texels a block keeps: all non-zero ones, those without STP (drawn solid when the primitive is
 * semi-transparent) or those with STP (blended). */
enum { VARIANT_ALL, VARIANT_SOLID, VARIANT_BLENDED };

static uint16_t to_pvr(uint16_t t)
{
    return t ? (uint16_t)(0x8000u | (t & 0x03e0u) | (t & 0x001fu) << 10 | (t >> 10 & 0x001fu)) : 0;
}

/* ---- textures ---- */

static bool overlaps(int ax0, int ay0, int ax1, int ay1, int bx0, int by0, int bx1, int by1)
{
    return ax0 < bx1 && bx0 < ax1 && ay0 < by1 && by0 < ay1;
}

static void convert_texture(Texture *t)
{
    const uint16_t *vram = gpu_vram();
    int depth = (int)(t->page >> 7 & 3);
    int px = (int)(t->page & 0xf) * 64, py = (int)(t->page >> 4 & 1) * 256;
    int cx = (int)(t->clut & 0x3f) * 16, cy = (int)(t->clut >> 6 & 0x1ff);
    uint16_t lut[256];
    if (depth < 2)
        for (int i = 0; i < (depth == 0 ? 16 : 256); i++)
            lut[i] = vram[cy * VRAM_W + ((cx + i) & (VRAM_W - 1))];
    int mask_x = (int)(t->tw & 31), mask_y = (int)(t->tw >> 5 & 31);
    int ofs_x = (int)(t->tw >> 10 & 31), ofs_y = (int)(t->tw >> 15 & 31);
    for (int v = 0; v < t->bh; v++) {
        int pv = t->bv + v;
        int tv = ((pv & ~(mask_y * 8)) | ((ofs_y & mask_y) * 8)) & 0xff;
        const uint16_t *row = vram + ((py + tv) & (VRAM_H - 1)) * VRAM_W;
        uint16_t *out = convert_buf + v * t->bw;
        if (t->tw == 0 && depth == 0) {
            for (int w = 0; w < t->bw / 4; w++) {
                uint16_t word = row[(px + t->bu / 4 + w) & (VRAM_W - 1)];
                out[w * 4] = lut[word & 15];
                out[w * 4 + 1] = lut[word >> 4 & 15];
                out[w * 4 + 2] = lut[word >> 8 & 15];
                out[w * 4 + 3] = lut[word >> 12];
            }
            continue;
        }
        for (int u = 0; u < t->bw; u++) {
            int pu = t->bu + u;
            int tu = ((pu & ~(mask_x * 8)) | ((ofs_x & mask_x) * 8)) & 0xff;
            if (depth == 0)
                out[u] = lut[row[(px + tu / 4) & (VRAM_W - 1)] >> ((tu & 3) * 4) & 15];
            else if (depth == 1)
                out[u] = lut[row[(px + tu / 2) & (VRAM_W - 1)] >> ((tu & 1) * 8) & 0xff];
            else
                out[u] = row[(px + tu) & (VRAM_W - 1)];
        }
    }
    /* PSX texels (bit 15 = STP) to PVR ones, keeping the variant's. */
    t->empty = true;
    for (int i = 0; i < t->bw * t->bh; i++) {
        uint16_t texel = convert_buf[i];
        bool keep = texel != 0 && (t->variant == VARIANT_ALL || (t->variant == VARIANT_SOLID) == !(texel & 0x8000));
        convert_buf[i] = keep ? to_pvr(texel) : 0;
        t->empty &= !keep;
    }
    pvr_txr_load(convert_buf, t->ptr, (size_t)(t->bw * t->bh * 2));
    conversions++;
}

static unsigned bucket_of(unsigned page, unsigned clut, unsigned tw, int variant)
{
    return (page * 31u + clut * 7u + tw * 13u + (clut >> 6) + (unsigned)variant * 101u) % BUCKETS;
}

static void free_texture(Texture *t)
{
    int index = (int)(t - textures);
    int *link = &buckets[bucket_of(t->page, t->clut, t->tw, t->variant)];
    while (*link != index)
        link = &textures[*link].next;
    *link = t->next;
    pvr_mem_free(t->ptr);
    texture_bytes -= (size_t)(t->bw * t->bh * 2);
    t->used = false;
    t->next = free_list;
    free_list = index;
}

/* The smallest aligned power-of-two block (8 at least) holding texels a..b. */
static int block_size(int a, int b)
{
    int size = 8;
    while (size < PAGE && a / size != b / size)
        size *= 2;
    return size;
}

/* Frees the least recently used block no frame draws with; false when there is none. */
static bool evict(void)
{
    Texture *pick = NULL;
    for (int i = 0; i < MAX_TEXTURES; i++) {
        Texture *t = &textures[i];
        if (t->used && t->refs == 0 && (!pick || t->last_use < pick->last_use))
            pick = t;
    }
    if (!pick)
        return false;
    free_texture(pick);
    return true;
}

/* The cached block holding texels (u0, v0)..(u1, v1) of the primitive's page, made if needed. */
static int texture_for(const HwState *s, int variant, int u0, int v0, int u1, int v1)
{
    int depth = (int)(s->texpage >> 7 & 3);
    if (depth == 3)
        depth = 2;
    unsigned page = (s->texpage & 0x1f) | (unsigned)depth << 7;
    unsigned clut = depth < 2 ? s->clut : 0;
    /* Neighbouring primitives mostly share a block. */
    static int last = -1;
    if (last >= 0) {
        Texture *t = &textures[last];
        if (t->used && !t->stale && t->page == page && t->clut == clut && t->tw == s->tw && t->variant == variant &&
            t->bu <= u0 && u1 < t->bu + t->bw && t->bv <= v0 && v1 < t->bv + t->bh) {
            t->last_use = frame_counter;
            return last;
        }
    }
    unsigned bucket = bucket_of(page, clut, s->tw, variant);
    for (int i = buckets[bucket]; i >= 0; i = textures[i].next) {
        Texture *t = &textures[i];
        if (!t->stale && t->page == page && t->clut == clut && t->tw == s->tw && t->variant == variant && t->bu <= u0 &&
            u1 < t->bu + t->bw && t->bv <= v0 && v1 < t->bv + t->bh) {
            t->last_use = frame_counter;
            last = i;
            return i;
        }
    }
    int bw = block_size(u0, u1), bh = block_size(v0, v1);
    size_t bytes = (size_t)(bw * bh * 2);
    if (free_list < 0 && !evict()) {
        failures++;
        return -1;
    }
    pvr_ptr_t ptr = NULL;
    while ((texture_bytes + bytes > TEXTURE_BUDGET || !(ptr = pvr_mem_malloc(bytes))) && evict())
        ;
    if (!ptr && texture_bytes + bytes <= TEXTURE_BUDGET)
        ptr = pvr_mem_malloc(bytes);
    if (!ptr) {
        failures++;
        return -1;
    }
    int pick = free_list;
    free_list = textures[pick].next;
    Texture *t = &textures[pick];
    memset(t, 0, sizeof(*t));
    t->used = true;
    t->page = page;
    t->clut = clut;
    t->tw = s->tw;
    t->variant = variant;
    t->bu = u0 & ~(bw - 1);
    t->bv = v0 & ~(bh - 1);
    t->bw = bw;
    t->bh = bh;
    t->ptr = ptr;
    t->next = buckets[bucket];
    buckets[bucket] = pick;
    texture_bytes += bytes;
    t->last_use = frame_counter;
    int words = depth == 0 ? 4 : depth == 1 ? 2 : 1; /* texels per VRAM word */
    t->x0 = (int)(page & 0xf) * 64;
    t->y0 = (int)(page >> 4 & 1) * 256;
    if (s->tw == 0) {
        t->x1 = t->x0 + (t->bu + bw + words - 1) / words;
        t->x0 += t->bu / words;
        t->y1 = t->y0 + t->bv + bh;
        t->y0 += t->bv;
    } else {
        t->x1 = t->x0 + PAGE / words;
        t->y1 = t->y0 + PAGE;
    }
    t->cx0 = (int)(clut & 0x3f) * 16;
    t->cy = (int)(clut >> 6 & 0x1ff);
    t->cx1 = depth < 2 ? t->cx0 + (depth == 0 ? 16 : 256) : t->cx0;
    convert_texture(t);
    last = pick;
    return pick;
}

/* Drops the texture references of a record's primitives first..last-1 (they are being thrown away). */
static void release(Slot *s, int first, int last)
{
    for (int i = first; i < last; i++) {
        int tex = s->prims[i].tex;
        if (tex >= 0 && --textures[tex].refs == 0 && textures[tex].stale)
            free_texture(&textures[tex]);
    }
}

void kos_gpu_stats(int *converted, int *failed)
{
    *converted = conversions;
    *failed = failures;
    conversions = failures = 0;
}

/* ---- recording ---- */

/* Compiled headers by what decides them (list, texture block, blend mode): a scene switches between a
 * few dozen, and compiling one each time cost more than drawing its polygon. */
#define HEADER_CACHE 64

static uint32_t header(pvr_list_t list, int tex, int mode, pvr_poly_hdr_t *dst)
{
    static struct {
        bool used;
        pvr_list_t list;
        pvr_ptr_t ptr;
        int w, h, mode;
        uint32_t id;
        pvr_poly_hdr_t hdr;
    } cache[HEADER_CACHE];
    static uint32_t compiled;
    pvr_ptr_t ptr = tex >= 0 ? textures[tex].ptr : NULL;
    int w = tex >= 0 ? textures[tex].bw : 0, h = tex >= 0 ? textures[tex].bh : 0;
    unsigned slot = ((unsigned)((uintptr_t)ptr >> 5) ^ (unsigned)list * 7u ^ (unsigned)(mode + 1) * 13u) % HEADER_CACHE;
    if (cache[slot].used && cache[slot].list == list && cache[slot].ptr == ptr && cache[slot].w == w &&
        cache[slot].h == h && cache[slot].mode == mode) {
        *dst = cache[slot].hdr;
        return cache[slot].id;
    }
    pvr_poly_cxt_t cxt;
    if (tex >= 0)
        pvr_poly_cxt_txr(&cxt, list, PVR_TXRFMT_ARGB1555 | PVR_TXRFMT_NONTWIDDLED, w, h, ptr, PVR_FILTER_NONE);
    else
        pvr_poly_cxt_col(&cxt, list);
    cxt.gen.culling = PVR_CULLING_NONE;
    cxt.depth.comparison = PVR_DEPTHCMP_GEQUAL;
    if (list == PVR_LIST_TR_POLY) {
        switch (mode) {
        case 0: cxt.blend.src = PVR_BLEND_SRCALPHA; cxt.blend.dst = PVR_BLEND_INVSRCALPHA; break;
        case 1:
        case 3: cxt.blend.src = PVR_BLEND_SRCALPHA; cxt.blend.dst = PVR_BLEND_ONE; break;
        default: cxt.blend.src = PVR_BLEND_ZERO; cxt.blend.dst = PVR_BLEND_INVDESTCOLOR; break;
        }
    }
    cache[slot].used = true;
    cache[slot].list = list;
    cache[slot].ptr = ptr;
    cache[slot].w = w;
    cache[slot].h = h;
    cache[slot].mode = mode;
    cache[slot].id = ++compiled;
    pvr_poly_compile(&cache[slot].hdr, &cxt);
    *dst = cache[slot].hdr;
    return cache[slot].id;
}

/* Texture coordinates of the primitive being added: texel (u, v) is at ((u - u0) * su, (v - v0) * sv). */
static float map_u0, map_v0, map_su, map_sv;

/* A new primitive at the end of the current record, its header set; NULL when it can't be drawn.
 * (u0, v0)..(u1, v1) are the texels it uses. */
static Prim *add_prim(const HwState *s, int variant, int *mode, int u0, int v0, int u1, int v1)
{
    if (!ready || cur->count >= MAX_PRIMS) {
        if (ready && !cur->full_reported) {
            cur->full_reported = true;
            printf("kos_gpu: more than %d primitives in a frame\n", MAX_PRIMS);
        }
        return NULL;
    }
    int tex = -1;
    if (s->textured) {
        tex = texture_for(s, variant, u0, v0, u1 > 255 ? 255 : u1, v1 > 255 ? 255 : v1);
        if (tex < 0 || textures[tex].empty)
            return NULL;
        textures[tex].refs++;
        map_u0 = (float)textures[tex].bu;
        map_v0 = (float)textures[tex].bv;
        map_su = 1.0f / (float)textures[tex].bw;
        map_sv = 1.0f / (float)textures[tex].bh;
    }
    bool blended = s->semi && (!s->textured || variant == VARIANT_BLENDED);
    *mode = blended ? (int)(s->texpage >> 5 & 3) : -1;
    pvr_list_t list = blended ? PVR_LIST_TR_POLY : tex >= 0 ? PVR_LIST_PT_POLY : PVR_LIST_OP_POLY;
    Prim *p = &cur->prims[cur->count];
    p->hdr_id = header(list, tex, *mode, &p->hdr);
    p->list = (uint8_t)list;
    p->tex = (int16_t)tex;
    cur->count++;
    cur->gen++;
    return p;
}

static uint32_t colour(const HwState *s, int mode, int r, int g, int b)
{
    static const uint32_t alpha[4] = {0x80, 0xff, 0xff, 0x40};
    if (s->textured) {
        if (s->raw) {
            r = g = b = 255;
        } else {
            r = r > 127 ? 255 : r * 2;
            g = g > 127 ? 255 : g * 2;
            b = b > 127 ? 255 : b * 2;
        }
    }
    uint32_t a = mode >= 0 ? alpha[mode] : 0xff;
    return a << 24 | (uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b;
}

static void vertex(pvr_vertex_t *pv, float x, float y, float u, float v, uint32_t argb, float z, bool last)
{
    pv->flags = last ? PVR_CMD_VERTEX_EOL : PVR_CMD_VERTEX;
    pv->x = (x - (float)cur->x) * cur->sx;
    pv->y = (y - (float)cur->y) * cur->sy;
    pv->z = z;
    pv->u = (u - map_u0) * map_su;
    pv->v = (v - map_v0) * map_sv;
    pv->argb = argb;
    pv->oargb = 0;
}

static float next_z(void)
{
    return (float)(cur->count + 2) * Z_STEP;
}

/* The GPU skips triangles with an edge of 1024 x 512 or more. */
static bool edge_ok(const HwVertex *a, const HwVertex *b)
{
    unsigned dx = (unsigned)(a->x - b->x + 1023), dy = (unsigned)(a->y - b->y + 511);
    return dx <= 2046 && dy <= 1022;
}

static bool drawable(const HwVertex *a, const HwVertex *b, const HwVertex *c)
{
    return edge_ok(a, b) && edge_ok(b, c) && edge_ok(c, a);
}

static void strip_variant(const HwState *s, int variant, const HwVertex *const *v, int count)
{
    int mode, u0 = 255, v0 = 255, u1 = 0, v1 = 0;
    for (int i = 0; i < count; i++) {
        u0 = v[i]->u < u0 ? v[i]->u : u0;
        u1 = v[i]->u > u1 ? v[i]->u : u1;
        v0 = v[i]->v < v0 ? v[i]->v : v0;
        v1 = v[i]->v > v1 ? v[i]->v : v1;
    }
    float z = next_z();
    Prim *p = add_prim(s, variant, &mode, u0, v0, u1, v1);
    if (!p)
        return;
    p->count = (uint8_t)count;
    for (int i = 0; i < count; i++)
        vertex(&p->v[i], (float)v[i]->x, (float)v[i]->y, v[i]->u, v[i]->v,
               colour(s, mode, v[i]->r, v[i]->g, v[i]->b), z, i == count - 1);
}

static void strip(const HwState *s, const HwVertex *const *v, int count)
{
    if (s->textured && s->semi) {
        strip_variant(s, VARIANT_SOLID, v, count);
        strip_variant(s, VARIANT_BLENDED, v, count);
    } else {
        strip_variant(s, VARIANT_ALL, v, count);
    }
}

void gpu_hw_polygon(const HwState *s, const HwVertex *v, int count)
{
    bool first = drawable(&v[0], &v[1], &v[2]);
    bool second = count == 4 && drawable(&v[1], &v[2], &v[3]);
    const HwVertex *all[4] = {&v[0], &v[1], &v[2], count == 4 ? &v[3] : NULL};
    if (first && (second || count == 3)) {
        strip(s, all, count);
    } else if (second) {
        strip(s, all + 1, 3);
    } else if (first) {
        strip(s, all, 3);
    }
}

static void rect_variant(const HwState *s, int variant, int x, int y, int w, int h, uint8_t r, uint8_t g,
                         uint8_t b, int u, int v)
{
    int mode;
    float z = next_z();
    Prim *p = add_prim(s, variant, &mode, u, v, u + w - 1, v + h - 1);
    if (!p)
        return;
    uint32_t c = colour(s, mode, r, g, b);
    p->count = 4;
    vertex(&p->v[0], (float)x, (float)y, (float)u, (float)v, c, z, false);
    vertex(&p->v[1], (float)(x + w), (float)y, (float)(u + w), (float)v, c, z, false);
    vertex(&p->v[2], (float)x, (float)(y + h), (float)u, (float)(v + h), c, z, false);
    vertex(&p->v[3], (float)(x + w), (float)(y + h), (float)(u + w), (float)(v + h), c, z, true);
}

void gpu_hw_rect(const HwState *s, int x, int y, int w, int h, uint8_t r, uint8_t g, uint8_t b, int u, int v)
{
    if (w <= 0 || h <= 0)
        return;
    if (s->textured && s->semi) {
        rect_variant(s, VARIANT_SOLID, x, y, w, h, r, g, b, u, v);
        rect_variant(s, VARIANT_BLENDED, x, y, w, h, r, g, b, u, v);
    } else {
        rect_variant(s, VARIANT_ALL, x, y, w, h, r, g, b, u, v);
    }
}

/* A line as a quad one PSX pixel thick across its major direction. */
void gpu_hw_line(const HwState *s, const HwVertex *a, const HwVertex *b)
{
    int dx = b->x - a->x, dy = b->y - a->y;
    if (dx > 1023 || dx < -1023 || dy > 511 || dy < -511)
        return;
    int mode;
    float z = next_z();
    Prim *p = add_prim(s, VARIANT_ALL, &mode, 0, 0, 0, 0);
    if (!p)
        return;
    uint32_t ca = colour(s, mode, a->r, a->g, a->b), cb = colour(s, mode, b->r, b->g, b->b);
    bool across_y = (dx < 0 ? -dx : dx) >= (dy < 0 ? -dy : dy);
    float ox = across_y ? 0.0f : 1.0f, oy = across_y ? 1.0f : 0.0f;
    float ax = (float)a->x, ay = (float)a->y, bx = (float)b->x + (across_y ? 1.0f : 0.0f),
          by = (float)b->y + (across_y ? 0.0f : 1.0f);
    p->count = 4;
    vertex(&p->v[0], ax, ay, 0, 0, ca, z, false);
    vertex(&p->v[1], ax + ox, ay + oy, 0, 0, ca, z, false);
    vertex(&p->v[2], bx, by, 0, 0, cb, z, false);
    vertex(&p->v[3], bx + ox, by + oy, 0, 0, cb, z, true);
}

/* ---- screens ---- */

static Slot *find_slot(int x, int y)
{
    for (int i = 0; i < MAX_SLOTS; i++)
        if (slots[i].used && slots[i].x == x && slots[i].y == y)
            return &slots[i];
    return NULL;
}

static Slot *new_slot(int x, int y, int w, int h)
{
    Slot *pick = NULL;
    for (int i = 0; i < MAX_SLOTS && !pick; i++)
        if (!slots[i].used)
            pick = &slots[i];
    for (int i = 0; i < MAX_SLOTS && !pick; i++)
        if (&slots[i] != cur && (slots[i].x != disp.x || slots[i].y != disp.y))
            pick = &slots[i];
    if (!pick)
        pick = &slots[0];
    release(pick, 0, pick->count);
    Prim *prims = pick->prims;
    pvr_ptr_t backdrop = pick->backdrop;
    int backdrop_h = pick->backdrop_h;
    memset(pick, 0, sizeof(*pick));
    pick->prims = prims;
    pick->backdrop = backdrop;
    pick->backdrop_h = backdrop_h;
    pick->used = true;
    pick->x = x;
    pick->y = y;
    pick->w = w;
    pick->h = h;
    pick->sx = SCREEN_W / (float)w;
    pick->sy = SCREEN_H / (float)h;
    pick->gen = 1;
    pick->cpu_gen = 1; /* whatever VRAM holds there now is the background */
    return pick;
}

/* Ends a pending frame start: the primitives after the mark move down to replace the last frame (or, when
 * `keep` is set, to follow its base), their depths renumbered for their new places. */
static void settle(Slot *s, bool keep)
{
    s->pending = false;
    if (keep && !s->single) {
        s->base = s->mark;
        s->single = true;
    } else if (!keep) {
        s->base = 0;
        s->single = false;
    }
    release(s, s->base, s->mark);
    int moved = s->count - s->mark;
    if (s->mark != s->base)
        memmove(&s->prims[s->base], &s->prims[s->mark], sizeof(Prim) * (size_t)moved);
    s->count = s->base + moved;
    for (int i = s->base; i < s->count; i++)
        for (int k = 0; k < s->prims[i].count; k++)
            s->prims[i].v[k].z = (float)(i + 2) * Z_STEP;
    s->gen++;
}

void gpu_hw_draw_env(int x1, int y1, int x2, int y2)
{
    if (!ready) {
        static bool tried;
        if (tried)
            return;
        tried = true;
        init();
        if (!ready)
            return;
    }
    int w = x2 - x1 + 1, h = y2 - y1 + 1;
    if (disp.w <= 0 || disp.rgb24 || w < disp.w || h + 16 < disp.h || w > BACKDROP || h > BACKDROP) {
        cur = NULL;
        return;
    }
    Slot *slot = find_slot(x1, y1);
    if (slot && (slot->w != w || slot->h != h))
        slot = NULL;
    if (!slot)
        slot = new_slot(x1, y1, w, h);
    /* A buffer that has been shown starts a new frame. Whether the last one goes (double buffering) or
     * stays as the base drawn over (the pause menu draws into the buffer on screen) shows at the next
     * PutDispEnv, so the new primitives go after a mark until then. */
    if (slot->pending)
        settle(slot, false);
    if (slot->shown) {
        slot->mark = slot->count;
        slot->pending = true;
        slot->shown = false;
        slot->full_reported = false;
    }
    cur = slot;
}

void gpu_hw_disp_env(int x, int y, int w, int h, bool rgb24)
{
    disp.x = x;
    disp.y = y;
    disp.w = w;
    disp.h = h;
    disp.rgb24 = rgb24;
    if (cur && cur->pending)
        settle(cur, cur->x == x && cur->y == y);
    Slot *slot = find_slot(x, y);
    if (slot) {
        slot->shown = true;
        slot->last_use = frame_counter;
    }
    if (rgb24)
        cur = NULL;
}

bool gpu_hw_drawing(void)
{
    return cur != NULL;
}

void gpu_hw_vram_written(int x, int y, int w, int h, bool cpu)
{
    int x1 = x + w, y1 = y + h;
    for (int i = 0; i < MAX_TEXTURES; i++) {
        Texture *t = &textures[i];
        if (t->used && !t->stale &&
            (overlaps(x, y, x1, y1, t->x0, t->y0, t->x1, t->y1) ||
             (t->cx1 > t->cx0 && overlaps(x, y, x1, y1, t->cx0, t->cy, t->cx1, t->cy + 1)))) {
            t->stale = true;
            if (t->refs == 0)
                free_texture(t);
        }
    }
    if (!cpu)
        return;
    for (int i = 0; i < MAX_SLOTS; i++) {
        Slot *s = &slots[i];
        if (s->used && overlaps(x, y, x1, y1, s->x, s->y, s->x + s->w, s->y + s->h))
            s->cpu_gen++;
    }
}

void gpu_hw_move(int sx, int sy, int w, int h, int dx, int dy)
{
    Slot *src = find_slot(sx, sy);
    if (!src || w < src->w || h + 16 < src->h)
        return;
    Slot *dst = find_slot(dx, dy);
    if (dst && (dst->w != src->w || dst->h != src->h))
        dst = NULL;
    if (!dst)
        dst = new_slot(dx, dy, src->w, src->h);
    if (dst == src)
        return;
    int count = src->pending ? src->mark : src->count;
    release(dst, 0, dst->count);
    memcpy(dst->prims, src->prims, sizeof(Prim) * (size_t)count);
    for (int i = 0; i < count; i++)
        if (dst->prims[i].tex >= 0)
            textures[dst->prims[i].tex].refs++;
    dst->count = dst->base = count;
    dst->single = dst->pending = false;
    dst->shown = true; /* a copy kept aside (the pause menu's): drawing into it later starts afresh */
    dst->gen++;
}

/* ---- showing a record ---- */

static void update_backdrop(Slot *s)
{
    s->backdrop_gen = s->cpu_gen;
    const uint16_t *vram = gpu_vram();
    uint16_t first = vram[s->y * VRAM_W + s->x];
    bool uniform = true;
    for (int y = 0; y < s->h && uniform; y++) {
        const uint16_t *row = vram + ((s->y + y) & (VRAM_H - 1)) * VRAM_W;
        for (int x = 0; x < s->w; x++)
            if (row[(s->x + x) & (VRAM_W - 1)] != first) {
                uniform = false;
                break;
            }
    }
    s->backdrop_solid = uniform;
    s->backdrop_colour = first;
    if (uniform)
        return;
    int need_h = s->h <= BACKDROP / 2 ? BACKDROP / 2 : BACKDROP;
    if (s->backdrop && s->backdrop_h < need_h) {
        pvr_mem_free(s->backdrop);
        s->backdrop = NULL;
    }
    if (!s->backdrop) {
        s->backdrop = pvr_mem_malloc((size_t)(BACKDROP * need_h * 2));
        s->backdrop_h = need_h;
        if (!s->backdrop) {
            printf("kos_gpu: no PVR memory for a background\n");
            s->backdrop_solid = true;
            return;
        }
    }
    uint16_t *dst = s->backdrop;
    for (int y = 0; y < s->h; y++) {
        const uint16_t *row = vram + ((s->y + y) & (VRAM_H - 1)) * VRAM_W;
        for (int x = 0; x < s->w; x++)
            row_buf[x] = (uint16_t)(to_pvr(row[(s->x + x) & (VRAM_W - 1)]) | 0x8000);
        pvr_txr_load(row_buf, dst + y * BACKDROP, (size_t)((s->w * 2 + 31) & ~31));
    }
}

static void draw_backdrop(const Slot *s)
{
    pvr_poly_cxt_t cxt;
    pvr_poly_hdr_t hdr;
    uint32_t argb = 0xffffffff;
    float u1 = 0, v1 = 0;
    if (s->backdrop_solid) {
        uint16_t c = s->backdrop_colour;
        argb = 0xff000000u | (uint32_t)(c & 31) << 19 | (uint32_t)(c >> 5 & 31) << 11 | (uint32_t)(c >> 10 & 31) << 3;
        pvr_poly_cxt_col(&cxt, PVR_LIST_OP_POLY);
    } else {
        pvr_poly_cxt_txr(&cxt, PVR_LIST_OP_POLY, PVR_TXRFMT_ARGB1555 | PVR_TXRFMT_NONTWIDDLED, BACKDROP,
                         s->backdrop_h, s->backdrop, PVR_FILTER_NONE);
        cxt.txr.env = PVR_TXRENV_REPLACE;
        u1 = (float)s->w / BACKDROP;
        v1 = (float)s->h / (float)s->backdrop_h;
    }
    cxt.gen.culling = PVR_CULLING_NONE;
    cxt.depth.comparison = PVR_DEPTHCMP_ALWAYS;
    pvr_poly_compile(&hdr, &cxt);
    pvr_prim(&hdr, sizeof(hdr));
    const float corner[4][2] = {{0, 0}, {1, 0}, {0, 1}, {1, 1}};
    pvr_vertex_t v;
    for (int i = 0; i < 4; i++) {
        v.flags = i == 3 ? PVR_CMD_VERTEX_EOL : PVR_CMD_VERTEX;
        v.x = corner[i][0] * SCREEN_W;
        v.y = corner[i][1] * SCREEN_H;
        v.z = Z_STEP;
        v.u = corner[i][0] * u1;
        v.v = corner[i][1] * v1;
        v.argb = argb;
        v.oargb = 0;
        pvr_prim(&v, sizeof(v));
    }
}

static void submit(const Slot *s, pvr_list_t list)
{
    pvr_list_begin(list);
    if (list == PVR_LIST_OP_POLY)
        draw_backdrop(s);
    uint32_t last = 0;
    int visible = s->pending ? s->mark : s->count; /* a frame begun over it isn't finished */
    for (int i = 0; i < visible; i++) {
        const Prim *p = &s->prims[i];
        if (p->list != list)
            continue;
        if (p->hdr_id != last) {
            pvr_prim(&p->hdr, sizeof(pvr_vertex_t) * (size_t)(p->count + 1));
            last = p->hdr_id;
        } else {
            pvr_prim(p->v, sizeof(pvr_vertex_t) * p->count);
        }
    }
    pvr_list_finish();
}

bool gpu_hw_present(int x, int y, int w, int h)
{
    (void)w, (void)h;
    if (!ready || disp.rgb24)
        return false;
    Slot *s = find_slot(x, y);
    if (!s)
        return false;
    if (s->gen == s->rendered_gen && s->backdrop_gen == s->cpu_gen)
        return true;
    if (pvr_check_ready() < 0)
        return true; /* still busy: tried again at the next vblank */
    frame_counter++;
    if (s->backdrop_gen != s->cpu_gen)
        update_backdrop(s);
    pvr_scene_begin();
    submit(s, PVR_LIST_OP_POLY);
    submit(s, PVR_LIST_PT_POLY);
    submit(s, PVR_LIST_TR_POLY);
    pvr_scene_finish();
    s->rendered_gen = s->gen;
    return true;
}
