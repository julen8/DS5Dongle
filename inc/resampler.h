#pragma once

#include <stdint.h>

typedef struct {
    /* === 配置：init() 设定，reset() 不动 === */
    uint64_t phaseStep; /* Q32 步进 = (in/out) * 2^32 */

    /* === 播放状态：reset() 清掉 === */
    int64_t phase; /* Q32 有符号读指针 */
    int16_t lastL; /* 上一块尾帧 左 — 跨块 x[-1] */
    int16_t lastR; /* 上一块尾帧 右 */
    bool haveLast;
} lerpResampler;

void lerpResamplerInit(lerpResampler *rs, uint32_t in_rate, uint32_t out_rate);

/* clean / 复位: 清播放状态, 保留 ratio/nch
   适用场景: 音乐停止后重新播放 / 切歌 / seek / 暂停后从头继续
*/
void lerpResamplerReset(lerpResampler *rs);

/* feed-mode 处理:
     in[]    : 交错 int16 PCM, nin 帧 (一帧 = nch 个样本)
     out[]   : 交错 int16 PCM 输出缓冲, 最多写 nout_max 帧
     返回值  : 实际输出帧数
*/
int lerpResamplerProcessInt16Out(lerpResampler *rs, const int16_t *in, int nin, int16_t *out, int nout_max);

/* feed-mode 处理:
     in[]    : 交错 int16 PCM, nin 帧 (一帧 = nch 个样本)
     out[]   : 交错 int8 PCM 输出缓冲, 最多写 nout_max 帧
     返回值  : 实际输出帧数
*/
int lerpResamplerProcessInt8Out(lerpResampler *rs, const int16_t *in, int nin, int8_t *out, int nout_max);
