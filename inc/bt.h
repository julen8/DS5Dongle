//
// Created by awalol on 2026/3/4.
//

#pragma once

#include <cstdint>
#include <vector>

enum CHANNEL_TYPE { INTERRUPT, CONTROL };

typedef void (*bt_data_callback_t)(CHANNEL_TYPE channel, uint8_t *data, uint16_t len);

int bt_init();
void bt_register_data_callback(bt_data_callback_t callback);
std::vector<uint8_t> get_feature_data(uint8_t reportId, uint16_t len);
void init_feature();
void set_feature_data(uint8_t reportId, uint8_t *data, uint16_t len);
void btRequestSend();
