#pragma once

#include <stdint.h>

#define USE_FLOAT_API 0
#define LERP_RS_MAX_CH 2 /* 最多支持的声道数 */

#if USE_FLOAT_API

typedef struct {
    /* ===== 配置：init 时设定，reset 不会改 ===== */
    double ratio; /* in_rate / out_rate */
    int nch;

    /* ===== 播放状态：reset 会清掉 ===== */
    double srcPos;              /* 小数读指针，单位 = 输入帧 */
    float last[LERP_RS_MAX_CH]; /* 上一块最末帧，用作跨块的 x[-1] */
    bool haveLast;
} lerpRs;

/* 一次性初始化（程序启动时 / 改变声道数时调用） */
void lerpRsInit(lerpRs *rs, double in_rate, double out_rate, int nch);

/* ★ clean / 复位 API：
   用于"音乐停止后重新播放"、"切歌"、"seek 跳转"、"暂停后从头继续"等场景。
   只清播放状态，保留 ratio / nch 配置，调用方不必重新 init。
   语义对齐 WDL_Resampler::Reset()。                                         */
void lerpRsReset(lerpRs *rs);

/* 运行时改变速率比例（不会清状态；适合 varispeed 平滑变速） */
void lerpRsSetRates(lerpRs *rs, double in_rate, double out_rate);

/* feed-mode 处理：
     in[]      —— interleaved 输入，nin 帧
     out[]     —— interleaved 输出缓冲，最多写 nout_max 帧
     返回值    —— 实际输出帧数
   建议 nin ≈ ceil(nout_max * ratio)，避免输入被丢弃。
*/
int lerpRsProcess(lerpRs *rs, const float *in, int nin, float *out, int nout_max);

#else

typedef struct {
    /* === 配置：init() 设定，reset() 不动 === */
    uint64_t phaseStep; /* Q32 步进 = (in/out) * 2^32 */
    int nch;

    /* === 播放状态：reset() 清掉 === */
    int64_t phase;                /* Q32 有符号读指针 */
    int16_t last[LERP_RS_MAX_CH]; /* 上一块的尾帧, 跨块插值用作 x[-1] */
    bool haveLast;
} lerpRs;

/* 用整数 Hz 配置 */
void lerpRsInit(lerpRs *rs, uint32_t in_rate_hz, uint32_t out_rate_hz, int nch);

/* clean / 复位: 清播放状态, 保留 ratio/nch
   适用场景: 音乐停止后重新播放 / 切歌 / seek / 暂停后从头继续 */
void lerpRsReset(lerpRs *rs);

/* 运行时改速率比 (不清状态; varispeed 用) */
void lerpRsSetRates(lerpRs *rs, uint32_t in_rate_hz, uint32_t out_rate_hz);

/* feed-mode 处理:
     in[]    : 交错 int16 PCM, nin 帧 (一帧 = nch 个样本)
     out[]   : 交错 int16 PCM 输出缓冲, 最多写 nout_max 帧
     返回值  : 实际输出帧数
   建议 nin ≈ ceil(nout_max * in_rate / out_rate)
*/
int lerpRsProcess(lerpRs *rs, const int16_t *in, int nin, int16_t *out, int nout_max);

#endif
