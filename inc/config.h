#pragma once

#include <cmath>
#include <cstdint>

extern "C" {
struct ConfigType {
    bool plugHeadset = false;        // plug headset
    bool isDse = false;              // dse
    bool audioActive = false;        // audio active
    uint8_t inactiveTime = 60;       // [10,60] min
    uint8_t pollingRateMode = 1;     // 0: 250Hz, 1: 500Hz, 2: real-time
    uint8_t audioBufferLength = 48;  // [16,128]
    uint8_t controllerMode = 2;      // 0: DS5, 1: DSE, 2: Auto
    struct {
        uint8_t speaker = 0;
        uint8_t microphone = 0;
    } mute = {};  // 0: SPEAKER(0x02) 1: MIC(0x05)
    struct {
        int16_t speaker = 0;
        int16_t microphone = 0;
    } volume = {};  // volume: [-25600, 0] , windows音量设置来的
};
}

extern ConfigType config;

// return [0, 1]
inline float getAudioGain() {
    static float lastSpeakerVolume = config.volume.speaker;
    static float audioGain = powf(10.0F, config.volume.speaker / 20.0F);

    if (lastSpeakerVolume != config.volume.speaker) {
        lastSpeakerVolume = config.volume.speaker;
        audioGain = powf(10.0F, config.volume.speaker / 20.0F);
    }

    return (config.mute.speaker == 0) ? audioGain : 0.0F;
}