#include "resampler.h"

#include <pico/critical_section.h>
#include <stddef.h>
#include <string.h>

/* (in/out) * 2^32 */
static inline uint64_t calcStep(uint32_t in_rate, uint32_t out_rate) {
    if (out_rate == 0) {
        return 0;
    }
    return ((uint64_t)in_rate << 32) / (uint64_t)out_rate;
}

/* int16 → int8: 右移 8 位 (保留高 8 bit, 符号扩展自动), 等价 /256 截断
   听感上等同对 PCM 做 -48 dB 量化, 这是 16→8 的标准做法
*/
static inline int8_t __not_in_flash_func(s16ToS8)(int16_t v) { return (int8_t)(v >> 8); }

/* 清播放状态, 保留配置 — 重播/切歌/seek 调这个 */
void lerpResamplerReset(lerpResampler *rs) {
    rs->phase = 0;
    rs->lastL = 0;
    rs->lastR = 0;
    rs->haveLast = false;
}

void lerpResamplerInit(lerpResampler *rs, uint32_t in_rate, uint32_t out_rate) {
    lerpResamplerReset(rs);
    rs->phaseStep = calcStep(in_rate, out_rate);
}

/* Q15 线性插值: y = a + ((b - a) * fq15) >> 15 (a, b ∈ int16)
   |b-a| <= 65535, fq15 <= 32767 → 乘积 ≤ 2^31 - 98303, 安全 fit int32
*/
static inline int16_t __not_in_flash_func(q15Lerp)(int16_t a16, int16_t b16, int32_t fq15) {
    const int32_t a = a16; /* 自动符号扩展 */
    const int32_t b = b16;
    return (int16_t)(a + (((b - a) * fq15) >> 15));
}

int __not_in_flash_func(lerpResamplerProcess)(lerpResampler *rs, const int16_t *in, int nin, void *out_ptr, int nout_max, bool isInt8) {
    const uint64_t step = rs->phaseStep;
    int64_t phase = rs->phase;
    int written = 0;

    if (nin < 0 || nout_max <= 0 || step == 0) {
        return 0;
    }

    /* ---- 阶段 1: phase 在 [-2^32, 0) — 用 last[] 作 x[-1], in[0] 作 x[0] ---- */
    while (written < nout_max && phase < 0) {
        if (!rs->haveLast || nin < 1) {
            goto done;
        }

        const uint32_t frac32 = (uint32_t)((uint64_t)(phase + ((int64_t)1 << 32)));
        const int32_t fq15 = (int32_t)(frac32 >> 17); /* 0..32767 */

        if (isInt8) {
            int8_t *out8 = (int8_t *)out_ptr;
            out8[0] = s16ToS8(q15Lerp(rs->lastL, in[0], fq15));
            out8[1] = s16ToS8(q15Lerp(rs->lastR, in[1], fq15));
            out_ptr = out8 + 2;
        } else {
            int16_t *out16 = (int16_t *)out_ptr;
            out16[0] = q15Lerp(rs->lastL, in[0], fq15);
            out16[1] = q15Lerp(rs->lastR, in[1], fq15);
            out_ptr = out16 + 2;
        }
        ++written;
        phase += (int64_t)step;
    }

    /* ---- 阶段 2: phase >= 0 — 全部从 in[] 内部插值, 热路径 ---- */
    while (written < nout_max) {
        const int32_t ipos = (int32_t)((uint64_t)phase >> 32);
        if (ipos + 1 >= nin) {
            break; /* 输入不够, 等下一次 */
        }

        const uint32_t frac32 = (uint32_t)((uint64_t)phase & 0xFFFFFFFFU);
        const int32_t fq15 = (int32_t)(frac32 >> 17);

        const int16_t *p0 = in + ((size_t)ipos * 2);
        const int16_t *p1 = p0 + 2;

        if (isInt8) {
            int8_t *out8 = (int8_t *)out_ptr;
            out8[0] = s16ToS8(q15Lerp(p0[0], p1[0], fq15));
            out8[1] = s16ToS8(q15Lerp(p0[1], p1[1], fq15));
            out_ptr = out8 + 2;
        } else {
            int16_t *out16 = (int16_t *)out_ptr;
            out16[0] = q15Lerp(p0[0], p1[0], fq15);
            out16[1] = q15Lerp(p0[1], p1[1], fq15);
            out_ptr = out16 + 2;
        }

        ++written;
        phase += (int64_t)step;
    }

done:
    /* 缓存这块尾帧, 下次跨块作 x[-1] 用 */
    if (nin > 0) {
        rs->lastL = in[((size_t)(nin - 1) * 2) + 0];
        rs->lastR = in[((size_t)(nin - 1) * 2) + 1];
        rs->haveLast = true;
    }

    /* 把 phase 移到下一块的坐标系 */
    phase -= (int64_t)nin << 32;
    /* 防御性夹紧: 用户若把过多输入塞进来, 不至于失控 */
    if (phase < -((int64_t)1 << 32)) {
        phase = -((int64_t)1 << 32);
    }

    rs->phase = phase;
    return written;
}

int __not_in_flash_func(lerpResamplerProcessInt16Out)(lerpResampler *rs, const int16_t *in, int nin, int16_t *out, int nout_max) { return lerpResamplerProcess(rs, in, nin, out, nout_max, false); }

int __not_in_flash_func(lerpResamplerProcessInt8Out)(lerpResampler *rs, const int16_t *in, int nin, int8_t *out, int nout_max) { return lerpResamplerProcess(rs, in, nin, out, nout_max, true); }
