#pragma once
/*
    PC(USB) -> DS5(BT)
*/

#include <stddef.h>
#include <stdint.h>

constexpr auto subPacketControlSize = 63;
constexpr auto subPacketHapticSize = 64;
constexpr auto subPacketAudioSize = 200;

enum subPacketType : uint8_t {
    subPacketTypeControl = 1,
    subPacketTypeHaptic,
    subPacketTypeAudio,
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
