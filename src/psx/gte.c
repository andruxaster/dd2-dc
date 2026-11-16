#include "psx/gte.h"

GteState gte;

/* ---- saturation and flags ---- */

static void set_flag(int bit)
{
    gte.flag |= 1u << bit;
}

/* Bit 31 is the OR of bits 30-23 and 18-13. */
static void finish_flags(void)
{
    if (gte.flag & 0x7f87e000u)
        gte.flag |= GTE_FLAG_ERROR;
}

/* MAC1..3 hold 44-bit results: flag overflow (bits 30-28) and underflow (27-25). The checks look at the
 * high word only (v >= 2^43 exactly when it is >= 800h): one 32-bit compare instead of a 64-bit one,
 * for every row of every vertex. */
static int64_t check_mac(int i, int64_t v)
{
    int32_t high = (int32_t)(v >> 32);
    if (high >= 0x800)
        set_flag(31 - i);
    else if (high < -0x800)
        set_flag(28 - i);
    return v;
}

/* sf selects the >> 12. Shifts by a constant: a 64-bit shift by a variable is a library call on 32-bit
 * CPUs (the SH4), and these run for every vertex. */
static int32_t set_mac(int i, int64_t v, int sf)
{
    check_mac(i, v);
    gte.mac[i] = (int32_t)(sf ? v >> 12 : v);
    return gte.mac[i];
}

static int32_t set_mac0(int64_t v)
{
    int32_t low = (int32_t)(uint32_t)v, high = (int32_t)(v >> 32);
    if (high != (low < 0 ? -1 : 0)) /* not a 32-bit value */
        set_flag(high < 0 ? 15 : 16);
    gte.mac[0] = low;
    return low;
}

/* IR1..3 clamp to -0x8000 (0 with lm) .. 0x7fff; bits 24-22. */
static int32_t lim_ir(int i, int32_t v, int lm)
{
    int32_t lo = lm ? 0 : -0x8000;
    if (v < lo) {
        set_flag(25 - i);
        return lo;
    }
    if (v > 0x7fff) {
        set_flag(25 - i);
        return 0x7fff;
    }
    return v;
}

static void set_ir_from_mac(int lm)
{
    for (int i = 1; i <= 3; i++)
        gte.ir[i] = lim_ir(i, gte.mac[i], lm);
}

static uint8_t lim_color(int bit, int32_t v)
{
    if (v < 0) {
        set_flag(bit);
        return 0;
    }
    if (v > 255) {
        set_flag(bit);
        return 255;
    }
    return (uint8_t)v;
}

/* Colour FIFO <- MAC / 16 with the code byte of RGBC. */
static void push_color(void)
{
    for (int i = 0; i < 4; i++) {
        gte.rgb_fifo[0][i] = gte.rgb_fifo[1][i];
        gte.rgb_fifo[1][i] = gte.rgb_fifo[2][i];
    }
    gte.rgb_fifo[2][0] = lim_color(21, gte.mac[1] >> 4);
    gte.rgb_fifo[2][1] = lim_color(20, gte.mac[2] >> 4);
    gte.rgb_fifo[2][2] = lim_color(19, gte.mac[3] >> 4);
    gte.rgb_fifo[2][3] = gte.rgbc[3];
}

static void push_sz(int32_t z)
{
    gte.sz[0] = gte.sz[1];
    gte.sz[1] = gte.sz[2];
    gte.sz[2] = gte.sz[3];
    if (z < 0) {
        set_flag(18);
        z = 0;
    } else if (z > 0xffff) {
        set_flag(18);
        z = 0xffff;
    }
    gte.sz[3] = (uint16_t)z;
}

void gte_push_sxy(int16_t x, int16_t y)
{
    gte.sx[0] = gte.sx[1];
    gte.sy[0] = gte.sy[1];
    gte.sx[1] = gte.sx[2];
    gte.sy[1] = gte.sy[2];
    gte.sx[2] = x;
    gte.sy[2] = y;
}

static int16_t lim_sxy(int bit, int32_t v)
{
    if (v < -0x400) {
        set_flag(bit);
        return -0x400;
    }
    if (v > 0x3ff) {
        set_flag(bit);
        return 0x3ff;
    }
    return (int16_t)v;
}

static void set_ir0(int64_t mac0)
{
    int32_t v = (int32_t)(mac0 >> 12);
    if (v < 0) {
        set_flag(12);
        v = 0;
    } else if (v > 0x1000) {
        set_flag(12);
        v = 0x1000;
    }
    gte.ir[0] = v;
}

/* ---- the GTE's division: unsigned Newton-Raphson with a reciprocal table ---- */

static uint8_t unr_table[0x101];
static uint8_t leading_zeros[0x100]; /* of a byte, 8 for 0 */
static int unr_ready;

static uint32_t divide(uint16_t h, uint16_t sz3)
{
    if (!unr_ready) {
        for (int i = 0; i < 0x101; i++) {
            int v = (0x40000 / (i + 0x100) + 1) / 2 - 0x101;
            unr_table[i] = (uint8_t)(v < 0 ? 0 : v);
        }
        for (int i = 0; i < 0x100; i++) {
            int z = 0;
            while (z < 8 && !(i & (0x80 >> z)))
                z++;
            leading_zeros[i] = (uint8_t)z;
        }
        unr_ready = 1;
    }
    if ((uint32_t)h >= (uint32_t)sz3 * 2) {
        set_flag(17);
        return 0x1ffff;
    }
    /* sz3 > h / 2 here, so it isn't 0. */
    int z = sz3 >> 8 ? leading_zeros[sz3 >> 8] : 8 + leading_zeros[sz3];
    uint32_t n = (uint32_t)h << z; /* h < 2 * sz3, so below 2^17 << (16 - bits of sz3): 32 bits */
    uint32_t d = (uint32_t)sz3 << z;
    uint32_t u = unr_table[(d - 0x7fc0) >> 7] + 0x101;
    d = (0x2000080 - d * u) >> 8;
    d = (0x0000080 + d * u) >> 8;
    uint64_t q = ((uint64_t)n * d + 0x8000) >> 16;
    return q > 0x1ffff ? 0x1ffff : (uint32_t)q;
}

/* ---- transforms ---- */

/* MAC(i) = (T * 1000h + M * V) >> sf*12 with its 44-bit flags, in 32-bit steps: the sum is taken as
 * floor(sum / 1000h) and its low 12 bits. Each 16 x 16-bit product fits 32 bits, splits exactly into
 * p >> 12 and p & FFFh, and the 44-bit overflow is the high part leaving 32 bits. A 64-bit sum and
 * shift cost a 32-bit CPU (the SH4) about 75 instructions a row, for every vertex. */
static void set_mac_row(int i, int32_t t, const int16_t m[3], int32_t x, int32_t y, int32_t z, int sf)
{
    int32_t p0 = m[0] * x, p1 = m[1] * y, p2 = m[2] * z;
    int32_t low = (p0 & 0xfff) + (p1 & 0xfff) + (p2 & 0xfff);
    int32_t q = (p0 >> 12) + (p1 >> 12) + (p2 >> 12) + (low >> 12);
    uint32_t high = (uint32_t)t + (uint32_t)q; /* floor(sum / 1000h), wrapped to 32 bits */
    if (q > 0 && (int32_t)high < t)
        set_flag(31 - i);
    else if (q < 0 && (int32_t)high > t)
        set_flag(28 - i);
    gte.mac[i] = (int32_t)(sf ? high : high << 12 | (uint32_t)(low & 0xfff));
}

static void rtp(int v, int sf, int last)
{
    int32_t x = gte.vx[v], y = gte.vy[v], z = gte.vz[v];
    for (int i = 0; i < 3; i++)
        set_mac_row(i + 1, gte.tr[i], gte.rt[i], x, y, z, sf);
    set_ir_from_mac(0);
    push_sz(gte.mac[3] >> (sf ? 0 : 12));
    /* n is at most 1FFFFh: as a signed 32-bit factor the products are one widening multiply. */
    int32_t n = (int32_t)divide(gte.h, gte.sz[3]);
    int16_t sx = lim_sxy(14, set_mac0((int64_t)n * gte.ir[1] + gte.ofx) >> 16);
    int16_t sy = lim_sxy(13, set_mac0((int64_t)n * gte.ir[2] + gte.ofy) >> 16);
    gte_push_sxy(sx, sy);
    if (last)
        set_ir0(set_mac0((int64_t)n * gte.dqa + gte.dqb));
}

void gte_op_rtps(int sf)
{
    gte.flag = 0;
    rtp(0, sf, 1);
    finish_flags();
}

void gte_op_rtpt(int sf)
{
    gte.flag = 0;
    rtp(0, sf, 0);
    rtp(1, sf, 0);
    rtp(2, sf, 1);
    finish_flags();
}

void gte_op_nclip(void)
{
    gte.flag = 0;
    int64_t v = (int64_t)gte.sx[0] * gte.sy[1] + (int64_t)gte.sx[1] * gte.sy[2] + (int64_t)gte.sx[2] * gte.sy[0] -
                (int64_t)gte.sx[0] * gte.sy[2] - (int64_t)gte.sx[1] * gte.sy[0] - (int64_t)gte.sx[2] * gte.sy[1];
    set_mac0(v);
    finish_flags();
}

static void average_z(int64_t v)
{
    set_mac0(v);
    int32_t z = (int32_t)(v >> 12);
    if (z < 0) {
        set_flag(18);
        z = 0;
    } else if (z > 0xffff) {
        set_flag(18);
        z = 0xffff;
    }
    gte.otz = (uint16_t)z;
}

void gte_op_avsz3(void)
{
    gte.flag = 0;
    average_z((int64_t)gte.zsf3 * (gte.sz[1] + gte.sz[2] + gte.sz[3]));
    finish_flags();
}

void gte_op_avsz4(void)
{
    gte.flag = 0;
    average_z((int64_t)gte.zsf4 * (gte.sz[0] + gte.sz[1] + gte.sz[2] + gte.sz[3]));
    finish_flags();
}

/* MAC = (T * 0x1000 + M * V) >> sf*12, IR = lim(MAC) */
static void mat_vec(const int16_t m[3][3], const int32_t *t, int32_t vx, int32_t vy, int32_t vz, int sf, int lm)
{
    for (int i = 0; i < 3; i++)
        set_mac_row(i + 1, t ? t[i] : 0, m[i], vx, vy, vz, sf);
    set_ir_from_mac(lm);
}

void gte_op_mvmva(int sf, int lm, int matrix, int vector, int translation)
{
    gte.flag = 0;
    static const int32_t zero[3] = {0, 0, 0};
    const int16_t(*m)[3] = matrix == 0 ? gte.rt : matrix == 1 ? gte.llm : gte.lcm;
    const int32_t *t = translation == 0 ? gte.tr : translation == 1 ? gte.bk : translation == 2 ? gte.fc : zero;
    int32_t x, y, z;
    if (vector < 3) {
        x = gte.vx[vector];
        y = gte.vy[vector];
        z = gte.vz[vector];
    } else {
        x = (int16_t)gte.ir[1];
        y = (int16_t)gte.ir[2];
        z = (int16_t)gte.ir[3];
    }
    mat_vec(m, t, x, y, z, sf, lm);
    finish_flags();
}

void gte_op_op(int sf, int lm)
{
    gte.flag = 0;
    int64_t d1 = gte.rt[0][0], d2 = gte.rt[1][1], d3 = gte.rt[2][2];
    set_mac(1, gte.ir[3] * d2 - gte.ir[2] * d3, sf);
    set_mac(2, gte.ir[1] * d3 - gte.ir[3] * d1, sf);
    set_mac(3, gte.ir[2] * d1 - gte.ir[1] * d2, sf);
    set_ir_from_mac(lm);
    finish_flags();
}

void gte_op_sqr(int sf)
{
    gte.flag = 0;
    for (int i = 1; i <= 3; i++)
        set_mac(i, (int64_t)gte.ir[i] * gte.ir[i], sf);
    set_ir_from_mac(1);
    finish_flags();
}

/* ---- colour ---- */

/* MAC = MAC + (FC - MAC) * IR0, as the hardware does it: IR = lim((FC << 12 - MAC) >> sf*12). */
static void interpolate_far(int64_t mac[3], int sf)
{
    for (int i = 0; i < 3; i++) {
        int64_t v = ((int64_t)gte.fc[i] << 12) - mac[i];
        check_mac(i + 1, v);
        gte.ir[i + 1] = lim_ir(i + 1, (int32_t)(sf ? v >> 12 : v), 0);
    }
    for (int i = 0; i < 3; i++)
        mac[i] = check_mac(i + 1, (int64_t)gte.ir[i + 1] * gte.ir[0] + mac[i]);
}

static void finish_color(int64_t mac[3], int sf, int lm)
{
    for (int i = 0; i < 3; i++)
        set_mac(i + 1, mac[i], sf);
    set_ir_from_mac(lm);
    push_color();
}

static void color_times_ir(int64_t mac[3], const uint8_t rgb[3])
{
    for (int i = 0; i < 3; i++)
        mac[i] = check_mac(i + 1, ((int64_t)rgb[i] * gte.ir[i + 1]) << 4);
}

/* Light: IR = LLM * V, then IR = BK + LCM * IR. */
static void light(int v, int sf)
{
    mat_vec(gte.llm, 0, gte.vx[v], gte.vy[v], gte.vz[v], sf, 1);
    mat_vec(gte.lcm, gte.bk, (int16_t)gte.ir[1], (int16_t)gte.ir[2], (int16_t)gte.ir[3], sf, 1);
}

enum { COLOR_PLAIN, COLOR_MODULATE, COLOR_DEPTH };

static void normal_color(int v, int sf, int mode)
{
    light(v, sf);
    int64_t mac[3];
    if (mode == COLOR_PLAIN) {
        for (int i = 0; i < 3; i++)
            mac[i] = (sf ? (int64_t)gte.mac[i + 1] * 4096 : (int64_t)gte.mac[i + 1]);
    } else {
        color_times_ir(mac, gte.rgbc);
        if (mode == COLOR_DEPTH)
            interpolate_far(mac, sf);
    }
    finish_color(mac, sf, 1);
}

void gte_op_ncs(int sf) { gte.flag = 0; normal_color(0, sf, COLOR_PLAIN); finish_flags(); }
void gte_op_nct(int sf) { gte.flag = 0; for (int v = 0; v < 3; v++) normal_color(v, sf, COLOR_PLAIN); finish_flags(); }
void gte_op_nccs(int sf) { gte.flag = 0; normal_color(0, sf, COLOR_MODULATE); finish_flags(); }
void gte_op_ncct(int sf) { gte.flag = 0; for (int v = 0; v < 3; v++) normal_color(v, sf, COLOR_MODULATE); finish_flags(); }
void gte_op_ncds(int sf) { gte.flag = 0; normal_color(0, sf, COLOR_DEPTH); finish_flags(); }
void gte_op_ncdt(int sf) { gte.flag = 0; for (int v = 0; v < 3; v++) normal_color(v, sf, COLOR_DEPTH); finish_flags(); }

static void color_from_ir(int sf, int depth)
{
    mat_vec(gte.lcm, gte.bk, (int16_t)gte.ir[1], (int16_t)gte.ir[2], (int16_t)gte.ir[3], sf, 1);
    int64_t mac[3];
    color_times_ir(mac, gte.rgbc);
    if (depth)
        interpolate_far(mac, sf);
    finish_color(mac, sf, 1);
}

void gte_op_cc(int sf) { gte.flag = 0; color_from_ir(sf, 0); finish_flags(); }
void gte_op_cdp(int sf) { gte.flag = 0; color_from_ir(sf, 1); finish_flags(); }

static void depth_cue_rgb(const uint8_t rgb[3], int sf)
{
    int64_t mac[3];
    for (int i = 0; i < 3; i++)
        mac[i] = (int64_t)rgb[i] << 16;
    interpolate_far(mac, sf);
    finish_color(mac, sf, 0);
}

void gte_op_dpcs(int sf)
{
    gte.flag = 0;
    depth_cue_rgb(gte.rgbc, sf);
    finish_flags();
}

void gte_op_dpct(int sf)
{
    gte.flag = 0;
    for (int n = 0; n < 3; n++) {
        uint8_t rgb[3] = {gte.rgb_fifo[0][0], gte.rgb_fifo[0][1], gte.rgb_fifo[0][2]};
        depth_cue_rgb(rgb, sf);
    }
    finish_flags();
}

void gte_op_intpl(int sf)
{
    gte.flag = 0;
    int64_t mac[3];
    for (int i = 0; i < 3; i++)
        mac[i] = (int64_t)gte.ir[i + 1] << 12;
    interpolate_far(mac, sf);
    finish_color(mac, sf, 0);
    finish_flags();
}

void gte_op_dcpl(int sf)
{
    gte.flag = 0;
    int64_t mac[3];
    color_times_ir(mac, gte.rgbc);
    interpolate_far(mac, sf);
    finish_color(mac, sf, 0);
    finish_flags();
}

void gte_op_gpf(int sf)
{
    gte.flag = 0;
    int64_t mac[3];
    for (int i = 0; i < 3; i++)
        mac[i] = (int64_t)gte.ir[i + 1] * gte.ir[0];
    finish_color(mac, sf, 0);
    finish_flags();
}

void gte_op_gpl(int sf)
{
    gte.flag = 0;
    int64_t mac[3];
    for (int i = 0; i < 3; i++)
        mac[i] = check_mac(i + 1, ((sf ? (int64_t)gte.mac[i + 1] * 4096 : (int64_t)gte.mac[i + 1])) + (int64_t)gte.ir[i + 1] * gte.ir[0]);
    finish_color(mac, sf, 0);
    finish_flags();
}

/* ---- register side effects ---- */

void gte_set_irgb(uint32_t v)
{
    gte.ir[1] = (int32_t)(v & 0x1f) * 0x80;
    gte.ir[2] = (int32_t)((v >> 5) & 0x1f) * 0x80;
    gte.ir[3] = (int32_t)((v >> 10) & 0x1f) * 0x80;
}

uint32_t gte_get_orgb(void)
{
    uint32_t out = 0;
    for (int i = 0; i < 3; i++) {
        int32_t c = gte.ir[i + 1] >> 7;
        c = c < 0 ? 0 : c > 0x1f ? 0x1f : c;
        out |= (uint32_t)c << (5 * i);
    }
    return out;
}
