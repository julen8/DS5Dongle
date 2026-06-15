#include "lerp_resampler.h"

#include <math.h>
#include <pico/critical_section.h>
#include <stddef.h>
#include <string.h>

#if USE_FLOAT_API

void lerpRsInit(lerpRs *rs, double in_rate, double out_rate, int nch) {
    memset(rs, 0, sizeof(*rs));
    rs->ratio = in_rate / out_rate;
    rs->nch = nch;
    rs->haveLast = false;
}

/* 只清播放相关状态，保留 ratio / nch */
void lerpRsReset(lerpRs *rs) {
    rs->srcPos = 0.0;
    rs->haveLast = false;
    memset(rs->last, 0, sizeof(rs->last));
}

void lerpRsSetRates(lerpRs *rs, double in_rate, double out_rate) { rs->ratio = in_rate / out_rate; }

int lerpRsProcess(lerpRs *rs, const float *in, int nin, float *out, int nout_max) {
    const int nch = rs->nch;
    if (nch < 1 || nch > LERP_RS_MAX_CH || nin < 0 || nout_max <= 0) {
        return 0;
    }

    double pos = rs->srcPos;
    int written = 0;

    while (written < nout_max) {
        int ipos = 0;
        double frac = NAN;
        const float *p0 = nullptr;
        const float *p1 = nullptr;

        /* pos 短暂可能落入 [-1, 0)，发生在跨块边界（上一块的尾巴被存进 last[]）*/
        if (pos >= 0.0) {
            ipos = (int)pos;
            frac = pos - (double)ipos;
        } else {
            ipos = -1;
            frac = pos + 1.0; /* ∈ [0, 1) */
        }

        if (ipos < 0) {
            /* 用上块缓存的最末帧充当 x[-1] */
            if (!rs->haveLast || nin < 1) {
                break;
            }
            p0 = rs->last;
            p1 = in; /* in[0] */
        } else {
            if (ipos + 1 >= nin) {
                break; /* 输入不够，等下一次喂数据 */
            }
            p0 = in + (size_t)ipos * nch;
            p1 = in + (size_t)(ipos + 1) * nch;
        }

        /* y = p0*(1-frac) + p1*frac     */
        const double k = 1.0 - frac;
        for (int c = 0; c < nch; ++c) {
            out[c] = (float)(((double)p0[c] * k) + ((double)p1[c] * frac));
        }

        out += nch;
        ++written;
        pos += rs->ratio;
    }

    /* 记下这块的最末帧，供下次跨块插值用 */
    if (nin > 0) {
        memcpy(rs->last, in + ((size_t)(nin - 1) * nch), (size_t)nch * sizeof(float));
        rs->haveLast = true;
    }

    /* 把 pos 转换到下一块的坐标系 */
    double newPos = pos - (double)nin;
    if (newPos < -1.0) {
        newPos = -1.0; /* 防御：输入远多于 nout_max 时夹紧 */
    }
    rs->srcPos = newPos;

    return written;
}

#else

/* (in/out) * 2^32 */
static uint64_t calcStep(uint32_t in_rate, uint32_t out_rate) {
    if (out_rate == 0) {
        return 0;
    }

    return ((uint64_t)in_rate << 32) / (uint64_t)out_rate;
}

void lerpRsInit(lerpRs *rs, uint32_t in_rate_hz, uint32_t out_rate_hz, int nch) {
    memset(rs, 0, sizeof(*rs));
    rs->nch = nch;
    rs->phaseStep = calcStep(in_rate_hz, out_rate_hz);
    rs->haveLast = false;
}

/* ★ 清播放状态, 保留配置 — 重播/切歌/seek 调这个 */
void lerpRsReset(lerpRs *rs) {
    rs->phase = 0;
    rs->haveLast = false;
    memset(rs->last, 0, sizeof(rs->last));
}

void lerpRsSetRates(lerpRs *rs, uint32_t in_rate_hz, uint32_t out_rate_hz) { rs->phaseStep = calcStep(in_rate_hz, out_rate_hz); }

/* Q15 线性插值: y = a + ((b - a) * fq15) >> 15
   |b-a| <= 65535, fq15 <= 32767 → 乘积 ≤ 2^31 - 98303, 安全 fit int32       */
static inline int16_t __not_in_flash_func(q15Lerp)(int16_t a16, int16_t b16, int32_t fq15) {
    const int32_t a = a16; /* 自动符号扩展 */
    const int32_t b = b16;
    return (int16_t)(a + (((b - a) * fq15) >> 15));
}

int __not_in_flash_func(lerpRsProcess)(lerpRs *rs, const int16_t *in, int nin, int16_t *out, int nout_max) {
    const int nch = rs->nch;
    const uint64_t step = rs->phaseStep;
    int64_t phase = rs->phase;
    int written = 0;

    if (nch < 1 || nch > LERP_RS_MAX_CH || nin < 0 || nout_max <= 0 || step == 0) {
        return 0;
    }

    /* ---- 阶段 1: phase 在 [-2^32, 0) — 用 last[] 作 x[-1], in[0] 作 x[0] ---- */
    while (written < nout_max && phase < 0) {
        if (!rs->haveLast || nin < 1) {
            goto done;
        }

        const uint32_t frac32 = (uint32_t)((uint64_t)(phase + ((int64_t)1 << 32)));
        const int32_t fq15 = (int32_t)(frac32 >> 17); /* 0..32767 */

        if (nch == 2) { /* 立体声专用快路径 */
            out[0] = q15Lerp(rs->last[0], in[0], fq15);
            out[1] = q15Lerp(rs->last[1], in[1], fq15);
        } else if (nch == 1) { /* 单声道专用快路径 */
            out[0] = q15Lerp(rs->last[0], in[0], fq15);
        } else {
            for (int c = 0; c < nch; ++c) {
                out[c] = q15Lerp(rs->last[c], in[c], fq15);
            }
        }
        out += nch;
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

        const int16_t *p0 = in + ((size_t)ipos * nch);
        const int16_t *p1 = p0 + nch;

        if (nch == 2) {
            out[0] = q15Lerp(p0[0], p1[0], fq15);
            out[1] = q15Lerp(p0[1], p1[1], fq15);
        } else if (nch == 1) {
            out[0] = q15Lerp(p0[0], p1[0], fq15);
        } else {
            for (int c = 0; c < nch; ++c) {
                out[c] = q15Lerp(p0[c], p1[c], fq15);
            }
        }
        out += nch;
        ++written;
        phase += (int64_t)step;
    }

done:
    /* 缓存这块尾帧, 下次跨块作 x[-1] 用 */
    if (nin > 0) {
        memcpy(rs->last, in + ((size_t)(nin - 1) * nch), (size_t)nch * sizeof(int16_t));
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

#endif
