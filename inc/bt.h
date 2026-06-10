//
// Created by awalol on 2026/3/4.
//

#pragma once

#include <stdint.h>

enum CHANNEL_TYPE { INTERRUPT, CONTROL };

typedef void (*bt_data_callback_t)(enum CHANNEL_TYPE channel, const uint8_t *data, uint16_t len);

int btInit();
void btRegisterDataCallback(bt_data_callback_t callback);
uint16_t getFeatureData(uint8_t reportId, uint8_t *outBuf, uint16_t maxLen);
void initFeature();
void setFeatureData(uint8_t reportId, const uint8_t *data, uint16_t len);
void btRequestSend();
