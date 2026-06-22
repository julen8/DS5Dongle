#include "haptic_resampler.h"

#include <pico/flash.h>
#include <stddef.h>
#include <string.h>

/* (in/out) * 2^32, 纯整数 */
static uint64_t calc_step(uint32_t in_rate, uint32_t out_rate) {
    if (out_rate == 0) return 0;
    return ((uint64_t)in_rate << 32) / (uint64_t)out_rate;
}

/* int16 → int8: 右移 8 位 (保留高 8 bit, 符号扩展自动), 等价 /256 截断
   听感上等同对 PCM 做 -48 dB 量化, 这是 16→8 的标准做法              */
static inline int8_t __not_in_flash_func(s16_to_s8)(int16_t v) { return (int8_t)(v >> 8); }

/* Q15 线性插值: y = a + ((b-a) * fq15) >> 15  (a, b ∈ int16) */
static inline int16_t __not_in_flash_func(q15_lerp_s16)(int16_t a16, int16_t b16, int32_t fq15) {
    const int32_t a = a16;
    const int32_t b = b16;
    return (int16_t)(a + (((b - a) * fq15) >> 15));
}

void lerp_rs_48to3_init(lerp_rs_48to3_t *rs, uint32_t in_rate_hz, uint32_t out_rate_hz) {
    memset(rs, 0, sizeof(*rs));
    rs->phase_step = calc_step(in_rate_hz, out_rate_hz);
}

/* ★ 清播放状态, 保留 phase_step */
void lerp_rs_48to3_reset(lerp_rs_48to3_t *rs) {
    rs->phase = 0;
    rs->last_l = 0;
    rs->last_r = 0;
    rs->have_last = 0;
}

void lerp_rs_48to3_set_rates(lerp_rs_48to3_t *rs, uint32_t in_rate_hz, uint32_t out_rate_hz) { rs->phase_step = calc_step(in_rate_hz, out_rate_hz); }

int __not_in_flash_func(lerp_rs_48to3_process)(lerp_rs_48to3_t *rs, const int16_t *in, int nin, int8_t *out, int nout_max) {
    const uint64_t step = rs->phase_step;
    int64_t phase = rs->phase;
    int written = 0;

    if (nin < 0 || nout_max <= 0 || step == 0) return 0;

    /* ---- 阶段 1: phase 在 [-2^32, 0) — 用 last_? 作 x[-1], in[0] 作 x[0] ---- */
    while (written < nout_max && phase < 0) {
        if (!rs->have_last || nin < 1) goto done;

        const uint32_t frac32 = (uint32_t)((uint64_t)(phase + ((int64_t)1 << 32)));
        const int32_t fq15 = (int32_t)(frac32 >> 17); /* 0..32767 */

        const int16_t l16 = q15_lerp_s16(rs->last_l, in[0], fq15);
        const int16_t r16 = q15_lerp_s16(rs->last_r, in[1], fq15);

        out[0] = s16_to_s8(l16);
        out[1] = s16_to_s8(r16);
        out += 2;
        ++written;
        phase += (int64_t)step;
    }

    /* ---- 阶段 2: phase >= 0 — 全部从 in[] 内部插值, 热路径 ---- */
    while (written < nout_max) {
        const int32_t ipos = (int32_t)((uint64_t)phase >> 32);
        if (ipos + 1 >= nin) break; /* 输入不够 */

        const uint32_t frac32 = (uint32_t)((uint64_t)phase & 0xFFFFFFFFu);
        const int32_t fq15 = (int32_t)(frac32 >> 17);

        const int16_t *p0 = in + (size_t)ipos * 2;
        const int16_t *p1 = p0 + 2;

        /* 整数比 (例如 48000/3000=16) 时, fq15 恒为 0 — 编译器/CPU 会很快 */
        const int16_t l16 = q15_lerp_s16(p0[0], p1[0], fq15);
        const int16_t r16 = q15_lerp_s16(p0[1], p1[1], fq15);

        out[0] = s16_to_s8(l16);
        out[1] = s16_to_s8(r16);
        out += 2;
        ++written;
        phase += (int64_t)step;
    }

done:
    /* 缓存这块尾帧, 下次跨块作 x[-1] */
    if (nin > 0) {
        rs->last_l = in[(size_t)(nin - 1) * 2 + 0];
        rs->last_r = in[(size_t)(nin - 1) * 2 + 1];
        rs->have_last = 1;
    }

    /* phase 搬到下一块坐标系 */
    phase -= (int64_t)nin << 32;
    if (phase < -((int64_t)1 << 32)) phase = -((int64_t)1 << 32);

    rs->phase = phase;
    return written;
}