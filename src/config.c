#include "config.h"

#include <math.h>
#include <pico/flash.h>

struct ConfigType config = CONFIG_DEFAULTS;

// return [0, 100]
uint8_t __not_in_flash_func(getSpeakerVolume)() {
    static int16_t lastSpeakerVolume = 9527;
    static float audioGain = 1.0F;

    if (lastSpeakerVolume != config.volume.speaker) {
        lastSpeakerVolume = config.volume.speaker;
        audioGain = powf(10.0F, ((float)config.volume.speaker) / 256.0F / 20.0F) * 100.0F;
    }

    return (config.mute.speaker == 0) ? (uint8_t)audioGain : 0;
}

// return [0, 100]
uint8_t __not_in_flash_func(getMicVolume)() {
    static int16_t lastMicVolume = 9527;
    static float audioGain = 1.0F;

    if (lastMicVolume != config.volume.microphone) {
        lastMicVolume = config.volume.microphone;
        audioGain = powf(10.0F, ((float)config.volume.microphone) / 256.0F / 20.0F) * 100.0F;
    }

    return (config.mute.microphone == 0) ? (uint8_t)audioGain : 0;
}
