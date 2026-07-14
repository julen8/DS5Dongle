#pragma once
#include <stdint.h>

[[nodiscard("audioInit result should be checked")]] bool audioInit();
void audioLoop();
void micAddOpusQueue(uint8_t *data, uint16_t len);
