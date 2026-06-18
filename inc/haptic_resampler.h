#ifndef LERP_RESAMPLER_48TO3_H_
#define LERP_RESAMPLER_48TO3_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 固定: int16 stereo 输入 → int8 stereo 输出, 线性插值, feed-mode
   与 WDL: SetMode(true, 0, false); SetRates(48000, 3000); SetFeedMode(true); 等价 */

typedef struct {
    /* === 配置: init() 设定, reset() 不动 === */
    uint64_t phase_step;     /* Q32 步进 = (in_rate/out_rate) * 2^32 */

    /* === 播放状态: reset() 清掉 === */
    int64_t  phase;          /* Q32 有符号读指针 (帧为单位) */
    int16_t  last_l;         /* 上一块尾帧 左 — 跨块 x[-1] */
    int16_t  last_r;         /* 上一块尾帧 右 */
    int      have_last;
} lerp_rs_48to3_t;

/* 用整数 Hz 配置, 默认就传 (48000, 3000) */
void lerp_rs_48to3_init      (lerp_rs_48to3_t *rs,
                              uint32_t in_rate_hz, uint32_t out_rate_hz);

/* ★ clean / 复位: 音乐停止后重新播放 / 切歌 / seek 时调用 */
void lerp_rs_48to3_reset     (lerp_rs_48to3_t *rs);

/* 运行时改速率比 (不清状态) */
void lerp_rs_48to3_set_rates (lerp_rs_48to3_t *rs,
                              uint32_t in_rate_hz, uint32_t out_rate_hz);

/* feed-mode 处理:
     in[]    : 交错 int16 PCM, nin 帧 (一帧 = 2 个 int16 = 4 字节)
     out[]   : 交错 int8  PCM, 最多写 nout_max 帧 (一帧 = 2 个 int8 = 2 字节)
     返回值  : 实际输出帧数
   你的典型用法: nin = 48, nout_max = 3, 返回 3                       */
int  lerp_rs_48to3_process   (lerp_rs_48to3_t *rs,
                              const int16_t   *in,  int nin,
                              int8_t          *out, int nout_max);

#ifdef __cplusplus
}
#endif
#endif /* LERP_RESAMPLER_48TO3_H_ */