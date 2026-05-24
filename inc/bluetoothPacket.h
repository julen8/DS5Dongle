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

void bluetoothPacketInit();

uint8_t* getBufferForSubPacket(subPacketType type);
void writeSubPacket(uint8_t* buff, subPacketType type);
void freeSubPacket(uint8_t* buffer, subPacketType type);

uint8_t* getBluetoothRawPacket(size_t* size);
void freeBluetoothRawPacket(uint8_t* bluetoothRawPacket);

size_t getSubPacketSize();
using onWriteCallbackType = void (*)();
void setBluetoothSubPacketWriteCallback(onWriteCallbackType callback);
