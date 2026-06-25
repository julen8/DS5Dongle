#pragma once

#include <stdint.h>

struct ConfigType {
    bool plugHeadset;           // plug headset
    bool isDse;                 // dse
    bool audioActive;           // audio active
    bool micActive;             // mic active
    bool disableMic;            // disable mic
    uint8_t inactiveTime;       // [10,60] min
    uint8_t pollingRateMode;    // 0: 250Hz, 1: 500Hz, 2: real-time
    uint8_t audioBufferLength;  // [16,128]
    uint8_t controllerMode;     // 0: DS5, 1: DSE, 2: Auto
    float microphoneGain;
    struct {
        uint8_t speaker;
        uint8_t microphone;
    } mute;  // 0: SPEAKER(0x02) 1: MIC(0x05)
    struct {
        int16_t speaker;
        int16_t microphone;
    } volume;  // volume: [-25600, 0] , windows音量设置来的
};

#define CONFIG_DEFAULTS                            \
    {                                              \
        .plugHeadset = false,                      \
        .isDse = false,                            \
        .audioActive = false,                      \
        .micActive = false,                        \
        .disableMic = false,                       \
        .inactiveTime = 20,                        \
        .pollingRateMode = 2,                      \
        .audioBufferLength = 48,                   \
        .controllerMode = 2,                       \
        .microphoneGain = 1.5F,                    \
        .mute = {.speaker = 0, .microphone = 0},   \
        .volume = {.speaker = 0, .microphone = 0}, \
    }

extern struct ConfigType config;

uint8_t getSpeakerVolume();
uint8_t getMicVolume();
