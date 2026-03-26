//
// Created by awalol on 2026/3/5.
//

#ifndef DS5_BRIDGE_AUDIO_H
#define DS5_BRIDGE_AUDIO_H
#include <cstdint>

void audio_init();
void audio_loop();
void send_speaker(const uint8_t *data);

#endif //DS5_BRIDGE_AUDIO_H