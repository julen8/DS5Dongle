#include "config.h"

#include <math.h>

struct ConfigType config = CONFIG_DEFAULTS;

// return [0, 1]
float getAudioGain() {
    static int16_t lastSpeakerVolume = 9527;
    static float audioGain = 1.0F;

    if (lastSpeakerVolume != config.volume.speaker) {
        lastSpeakerVolume = config.volume.speaker;
        audioGain = powf(10.0F, ((float)config.volume.speaker) / 256.0F / 20.0F);
    }

    return (config.mute.speaker == 0) ? audioGain : 0.0F;
}
