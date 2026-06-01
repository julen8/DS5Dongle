#pragma once

#include <cstddef>
#include <cstdint>

constexpr auto subPacketStatusSize = 63;
constexpr auto subPacketHapticSize = 64;
constexpr auto subPacketAudioSize = 200;

enum class subPacketType : std::uint8_t {
    status = 1,
    haptic,
    audio,
};

static constexpr uint8_t stateInitData[subPacketStatusSize] = {
    0xfd, 0xf7, 0x0, 0x0,  0x50, 0x50,  // Headphones, Speaker
    0xff, 0x9,  0x0, 0x0F, 0x0,  0x0,  0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,  0x0,  0x0,  0x0, 0x0,
    0x0,  0x0,  0x0, 0x0,  0x0,  0x0,  0x0, 0x0, 0x0, 0x0, 0xa, 0x7, 0x0, 0x0, 0x2, 0x1, 0x00, 0xff, 0xd7, 0x00  // RGB LED: R, G, B (Nijika Color!)✨
};

void bluetoothPacketInit();

uint8_t* getBufferForSubPacket(subPacketType type);
void writeSubPacket(uint8_t* buff, subPacketType type);
void freeSubPacket(uint8_t* buffer, subPacketType type);

uint8_t* getBluetoothRawPacket(size_t* size);
void freeBluetoothRawPacket(uint8_t* bluetoothRawPacket);

bool hasBluetoothRawPacketCanSend();

void cleanAllCachedAudio();
void cleanAllCachedHaptic();
