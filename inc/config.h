#pragma once

#include <stdatomic.h>
#include <stdint.h>

static_assert(ATOMIC_BOOL_LOCK_FREE == 2, "cross-core audio flags require lock-free atomic_bool");

struct ConfigType {
    float microphoneGain;
    volatile bool plugHeadset;  // plug headset
    volatile bool isDse;        // dse
    atomic_bool audioActive;    // audio active, shared between cores
    atomic_bool micActive;      // mic active, shared between cores
    uint8_t inactiveTime;       // [10,60] min
    uint8_t pollingRateMode;    // 0: 250Hz, 1: 500Hz, 2: 1000Hz
    uint8_t audioBufferLength;  // [16,128]
    uint8_t controllerMode;     // 0: DS5, 1: DSE, 2: Auto
    struct {
        uint8_t speaker;
        uint8_t microphone;
    } mute;  // 0: SPEAKER(0x02) 1: MIC(0x05)
    struct {
        int16_t speaker;     // [-25600, 0] , windows音量设置来的
        int16_t microphone;  // [0, 12288]
    } volume;
};

#define CONFIG_DEFAULTS                            \
    {                                              \
        .microphoneGain = 1.4F,                    \
        .plugHeadset = false,                      \
        .isDse = false,                            \
        .audioActive = false,                      \
        .micActive = false,                        \
        .inactiveTime = 20,                        \
        .pollingRateMode = 2,                      \
        .audioBufferLength = 48,                   \
        .controllerMode = 2,                       \
        .mute = {.speaker = 0, .microphone = 0},   \
        .volume = {.speaker = 0, .microphone = 0}, \
    }

extern struct ConfigType config;

uint8_t getSpeakerVolume();
uint8_t getMicVolume();
