#pragma once
#include <stdint.h>

void audioInit();
void audioLoop();
void micAddOpusQueue(uint8_t *data, uint16_t len);