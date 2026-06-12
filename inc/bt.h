//
// Created by awalol on 2026/3/4.
//

#pragma once

#include <stdint.h>

int btInit();
uint16_t getFeatureData(uint8_t reportId, uint8_t *outBuf, uint16_t maxLen);
void initFeature();
void setFeatureData(uint8_t reportId, const uint8_t *data, uint16_t len);
void btRequestSend();
