#include "wdlResample.h"

#include <resample.h>

WDL_Resampler audioResampler;

void audioResamplerInit() {
    audioResampler.SetMode(true, 0, false);
    audioResampler.SetRates(51200, 48000);
    audioResampler.SetFeedMode(true);
    audioResampler.Prealloc(audioChannels, audioResamplerInputFrames, audioResamplerOutputFrames);
}

void audioResamplerReset() { audioResampler.Reset(); }

int audioResamplerPrepare(int req_samples, int nch, float **inbuffer) { return audioResampler.ResamplePrepare(req_samples, nch, inbuffer); }

int audioResamplerOut(float *out, int nsamples_in, int nsamples_out, int nch) { return audioResampler.ResampleOut(out, nsamples_in, nsamples_out, nch); }
