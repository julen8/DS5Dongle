#pragma once

#include <cstdint>

extern "C" {
struct ConfigType {
    float speakerVolume = 0;            // [-100,0]
    bool plugHeadset = false;           // plug headset
    bool isDse = false;                 // dse
    bool audioActive = false;           // audio active
    uint8_t inactiveTime = 60;          // [10,60] min
    uint8_t pollingRateMode = 1;        // 0: 250Hz, 1: 500Hz, 2: real-time
    uint8_t audioBufferLength = 48;     // [16,128]
    uint8_t controllerMode = 2;         // 0: DS5, 1: DSE, 2: Auto
    uint8_t mute[2];                    // 0: SPEAKER(0x02) 1: MIC(0x05)
    float volume[2] = {-100.0F, 0.0F};  // 0: SPEAKER(0x02) 1: MIC(0x05)
};
}

extern ConfigType config;
