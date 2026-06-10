#pragma once

#ifdef __cplusplus
extern "C" {
#endif

constexpr int audioChannels = 2;                // 固定不能修改
constexpr int audioResamplerInputFrames = 32;   // 512 / audioResamplerOutToOpusInCount
constexpr int audioResamplerOutputFrames = 30;  // 480 / audioResamplerOutToOpusInCount

void audioResamplerInit();
void audioResamplerReset();
int audioResamplerPrepare(int req_samples, int nch, float **inbuffer);
int audioResamplerOut(float *out, int nsamples_in, int nsamples_out, int nch);

#ifdef __cplusplus
}
#endif
