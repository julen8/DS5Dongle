#pragma once

#include <stddef.h>
#include <stdint.h>

constexpr auto subPacketStatusSize = 63;
constexpr auto subPacketHapticSize = 64;
constexpr auto subPacketAudioSize = 200;

enum subPacketType : uint8_t {
    subPacketTypeStatus = 1,
    subPacketTypeHaptic,
    subPacketTypeAudio,
};

static constexpr uint8_t stateInitData[subPacketStatusSize] = {
    0xfd, 0xf7, 0x0, 0x0,  0x50, 0x50,  // Headphones, Speaker
    0xff, 0x9,  0x0, 0x0F, 0x0,  0x0,  0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,  0x0,  0x0,  0x0, 0x0,
    0x0,  0x0,  0x0, 0x0,  0x0,  0x0,  0x0, 0x0, 0x0, 0x0, 0xa, 0x7, 0x0, 0x0, 0x2, 0x1, 0x00, 0xff, 0xd7, 0x00  // RGB LED: R, G, B (Nijika Color!)✨
};

void bluetoothPacketInit();

uint8_t* getBufferForSubPacket(enum subPacketType type);
void writeSubPacket(uint8_t* buff, enum subPacketType type);
void freeSubPacket(uint8_t* buffer, enum subPacketType type);

uint8_t* getBluetoothRawPacket(size_t* size);
void freeBluetoothRawPacket(uint8_t* bluetoothRawPacket);

bool hasBluetoothRawPacketCanSend();

void cleanAllCachedAudio();
void cleanAllCachedHaptic();
