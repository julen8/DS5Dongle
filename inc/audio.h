#pragma once
#include <stdint.h>

void audioInit();
void audioLoop();
void mic_add_queue(uint8_t *data, uint16_t len);