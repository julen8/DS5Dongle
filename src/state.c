#include "state.h"

#include <assert.h>
#include <pico/critical_section.h>
#include <pico/flash.h>
#include <string.h>

#include "bluetooth_packet.h"
#include "config.h"
#include "log.h"

static_assert(sizeof(struct Ds5StatePacket) == ds5StatePacketSize, "sizeof(struct Ds5StatePacket) must be 63 bytes long");
static_assert(sizeof(struct Ds5ControlPacket) == ds5ControlPacketSize, "sizeof(struct Ds5ControlPacket) must be 63 bytes long");

static union Ds5StateUnion statePacket = {
    .data =
        {
            0x7f, 0x7d, 0x7f, 0x7e, 0x00, 0x00, 0xa7, 0x08, 0x00, 0x00, 0x00, 0x52, 0x43, 0x30, 0x41, 0x01, 0x00, 0x0e, 0x00, 0xef, 0xff,
            0x03, 0x03, 0x7b, 0x1b, 0x18, 0xf0, 0xcc, 0x9c, 0x60, 0x00, 0xfc, 0x80, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x09,
            0x09, 0x00, 0x00, 0x00, 0x00, 0x00, 0xa7, 0xad, 0x60, 0x00, 0x29, 0x18, 0x00, 0x53, 0x9f, 0x28, 0x35, 0xa5, 0xa8, 0x0c, 0x8b,
        },
};

static union Ds5ControlUnion controlPacket = ds5ControlInitPacket;

void __not_in_flash_func(setStatePacket)(const union Ds5StateUnion *packet) {
    if (packet->packet.PluggedHeadphones != statePacket.packet.PluggedHeadphones) {
        config.plugHeadset = packet->packet.PluggedHeadphones != 0;
    }

    memcpy(statePacket.data, packet->data, ds5StatePacketSize);
}

union Ds5StateUnion *__not_in_flash_func(getStatePacket)() { return &statePacket; }

void __not_in_flash_func(setControlPacket)(const uint8_t *data, int size) {
    if (size < ds5ControlPayloadSize) {
        LOGE("setControlPacket size %d < %d", size, ds5ControlPayloadSize);
        return;
    }

    static_assert(subPacketControlSize == ds5ControlPacketSize, "subPacketControlSize != ds5ControlPacketSize");

    uint8_t *controlBuffer = getBufferForSubPacket(subPacketTypeControl);
    if (controlBuffer == nullptr) {
        LOGE("getBufferForSubPacket subPacketTypeControl");
        return;
    }

    memcpy(controlBuffer, data, size);
    if (subPacketControlSize > size) {
        memset(controlBuffer + size, 0, subPacketControlSize - size);  // zero padding
    }

    writeSubPacket(controlBuffer, subPacketTypeControl);

    memcpy(controlPacket.data, data, ds5ControlPayloadSize);
}

union Ds5ControlUnion *__not_in_flash_func(getControlPacket)() { return &controlPacket; }

void __not_in_flash_func(reSendControlPacket)() {
    uint8_t *controlBuffer = getBufferForSubPacket(subPacketTypeControl);
    if (controlBuffer == nullptr) {
        LOGE("getBufferForSubPacket subPacketTypeControl");
        return;
    }

    memcpy(controlBuffer, controlPacket.data, ds5ControlPayloadSize);
    static_assert(subPacketControlSize > ds5ControlPayloadSize);
    memset(controlBuffer + ds5ControlPayloadSize, 0, subPacketControlSize - ds5ControlPayloadSize);  // zero padding

    writeSubPacket(controlBuffer, subPacketTypeControl);
}

void __not_in_flash_func(updateVolume)() {
    uint8_t value = getSpeakerVolume();
    controlPacket.packet.AllowHeadphoneVolume = 1;
    controlPacket.packet.AllowSpeakerVolume = 1;
    controlPacket.packet.VolumeHeadphones = value;
    controlPacket.packet.VolumeSpeaker = value;

    reSendControlPacket();
}

void __not_in_flash_func(updateMicVolume)() {
    uint8_t value = getMicVolume();
    controlPacket.packet.AllowMicVolume = 1;
    controlPacket.packet.VolumeMic = value;

    reSendControlPacket();
}
