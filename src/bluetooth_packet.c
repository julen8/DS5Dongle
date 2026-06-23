#include "bluetooth_packet.h"

#include <assert.h>
#include <pico/critical_section.h>
#include <pico/util/queue.h>
#include <stdatomic.h>
#include <stdint.h>
#include <string.h>

#include "bt.h"
#include "config.h"
#include "crc32.h"
#include "log.h"
#include "state.h"
/*
一个蓝牙包结构:
//BT PACKET:
bluetooth Head        (1)  bluetoothRawPacketHeadSize
DS5 BT packet head    (2) ds5BluetoothPacketHeadSize

SUB-PACKET (1个或多个)

crc32                 (4) ds5BluetoothPacketCrc32Size
*/

/*
// DS5 SUB-PACKET:
subpacket head        (2)  subPacketHeadSize
subpacket content    (AudioSetup:7 , Haptic:64, control:63, Audio:200)
*/

/*
一个包的示例:
┌───────────────────────────────────────────────────────────┐
│   最外层 bluetoothRawPacketHeadSize: [0] = 0xA2            │
│  ┌────────────────────────────────────────────────────┐  │
│  │ 外层 BT Report (0x32, 142字节)                       │  │
│  │  [0]     = 0x32 (Report ID)                        │  │
│  │  [1]     = seq_no (高4bit) | tag (低4bit)           │  │
│  │  ┌──────────────────────────────────────────────┐  │  │
│  │  │ Sub-Packet #0: OUTPUT (0x10)，64字节          │  │  │
│  │  │  [0]    = 0x10                               │  │  │
│  │  │  [1-63] = dualsense_output_msg.data[63]      │  │  │
│  │  │           (rumble, triggers, LED, audio...)  │  │  │
│  │  └──────────────────────────────────────────────┘  │  │
│  │  ┌──────────────────────────────────────────────┐  │  │
│  │  │ Sub-Packet #1: AUDIO_SETUP (0x91)，9字节   │   │  │
│  │  │  [0] = 0x91  [1] = 0x07                     │   │  │
│  │  │  [2] = 0xFE  [3-6] = 0x00                   │   │  │
│  │  │  [7] = 0xFF  [8] = frame_counter            │   │  │
│  │  └──────────────────────────────────────────────┘  │  │
│  │  ┌──────────────────────────────────────────────┐  │  │
│  │  │ Sub-Packet #2: HAPTICS_GRANULE (0x92)，66字节│   │  │
│  │  │  [0] = 0x92  [1] = 0x40                     │   │  │
│  │  │  [2-65] = int8_t PCM stereo @ 3kHz          │   │  │
│  │  └──────────────────────────────────────────────┘  │  │
│  │  [138-141] = CRC32 (seed=0xA2, 4字节)               │  │
│  └────────────────────────────────────────────────────┘  │
└──────────────────────────────────────────────────────────┘
*/

constexpr int bluetoothRawPacketHeadSize = 1;
constexpr int ds5BluetoothPacketHeadSize = 2;

constexpr int subPacketHeadSize = 2;
constexpr int subPacketAudioSetupSize = 7;

constexpr int ds5BluetoothPacketCrc32Size = 4;

constexpr int subPacketBuffHapticCount = 2;
constexpr int subPacketBuffControlCount = 3;
constexpr int subPacketBuffAudioCount = 2;
constexpr int bluetoothRawPacketCount = 3;

#ifndef offsetof
#    define offsetof(TYPE, MEMBER) ((size_t)&((TYPE*)0)->MEMBER)
#endif

#define container_of(ptr, type, member) ({ (type*)((char*)(ptr) - offsetof(type, member)); })

struct SubPacketBufferHaptic {
    volatile bool inuse;
    uint8_t buf[subPacketHapticSize];
};
struct SubPacketBufferControl {
    volatile bool inuse;
    uint8_t buf[subPacketControlSize];
};
struct SubPacketBufferAudio {
    // audio 缓冲在 core1 分配、core0 释放，使用原子标志避免跨核竞争
    atomic_bool inuse;
    uint8_t buf[subPacketAudioSize];
};

/*
https://controllers.fandom.com/wiki/Sony_DualSense
REPORT_ID
reportID[1]	Size	Type	Note
0x31	77	Output	Set Controller State or Audio (Audio theoretical)
0x32	141	Output	Set Controller State and/or Audio (unconfirmed)
0x33	205	Output	Set Controller State and/or Audio (unconfirmed)
0x34	269	Output	Set Controller State and/or Audio (unconfirmed)
0x35	333	Output	Set Controller State and/or Audio (unconfirmed)
0x36	397	Output	Set Controller State and/or Audio (unconfirmed)
0x37	461	Output	Set Controller State and/or Audio (unconfirmed)
0x38	525	Output	Set Controller State and/or Audio (unconfirmed)
0x39	546	Output	Set Controller State and/or Audio (unconfirmed)
不包含 REPORT_ID 所以 size 需要+1
*/

// 包含bluetoothRawPacketHeadSize和REPORT_ID所以比上面看到的值+2
constexpr size_t bluetoothRawPacketDataSize0x31 = 78 + bluetoothRawPacketHeadSize;
constexpr size_t bluetoothRawPacketDataSize0x32 = 142 + bluetoothRawPacketHeadSize;
constexpr size_t bluetoothRawPacketDataSize0x33 = 206 + bluetoothRawPacketHeadSize;
constexpr size_t bluetoothRawPacketDataSize0x34 = 270 + bluetoothRawPacketHeadSize;
constexpr size_t bluetoothRawPacketDataSize0x35 = 334 + bluetoothRawPacketHeadSize;
constexpr size_t bluetoothRawPacketDataSize0x36 = 398 + bluetoothRawPacketHeadSize;
constexpr size_t bluetoothRawPacketDataSize0x37 = 462 + bluetoothRawPacketHeadSize;
constexpr size_t bluetoothRawPacketDataSize0x38 = 526 + bluetoothRawPacketHeadSize;
constexpr size_t bluetoothRawPacketDataSize0x39 = 547 + bluetoothRawPacketHeadSize;

const struct {
    uint8_t reportId;
    size_t dataSize;
} bluetoothRawPacketReportIDAndDataSizeTrack[] = {
    {0x31, bluetoothRawPacketDataSize0x31}, {0x32, bluetoothRawPacketDataSize0x32}, {0x33, bluetoothRawPacketDataSize0x33},
    {0x34, bluetoothRawPacketDataSize0x34}, {0x35, bluetoothRawPacketDataSize0x35}, {0x36, bluetoothRawPacketDataSize0x36},
    {0x37, bluetoothRawPacketDataSize0x37}, {0x38, bluetoothRawPacketDataSize0x38}, {0x39, bluetoothRawPacketDataSize0x39},
};

struct BluetoothRawPacket {
    size_t size;
    bool inuse;
    uint8_t data[bluetoothRawPacketDataSize0x39];
};

static struct {
    struct SubPacketBufferHaptic subPacketBufferHaptic[subPacketBuffHapticCount];
    struct SubPacketBufferControl subPacketBufferControl[subPacketBuffControlCount];
    struct SubPacketBufferAudio subPacketBufferAudio[subPacketBuffAudioCount];
    struct BluetoothRawPacket bluetoothRawPacket[bluetoothRawPacketCount];

    queue_t subPacketHapticQueue;
    queue_t subPacketControlQueue;
    queue_t subPacketAudioQueue;

    uint8_t reportSeqCounter;
    uint8_t packetCounter;
    bool needSendAudioSetup;
    bool needSendControl;
} bluetoothPacket = {};

void needSendAudioSetup() {
    bluetoothPacket.needSendAudioSetup = true;
    if (!config.audioActive) {
        btRequestSend();
    }
}

static inline uint8_t* __not_in_flash_func(tryGetSubPacketBufferHaptic)() {
    for (int i = 0; i < subPacketBuffHapticCount; i++) {
        if (!bluetoothPacket.subPacketBufferHaptic[i].inuse) {
            bluetoothPacket.subPacketBufferHaptic[i].inuse = true;
            return bluetoothPacket.subPacketBufferHaptic[i].buf;
        }
    }

    return nullptr;
}

void cleanAllCachedHaptic() {
    uint8_t* hapticData = nullptr;
    while (queue_try_remove(&bluetoothPacket.subPacketHapticQueue, &hapticData)) {
        if (hapticData != nullptr) {
            freeSubPacket(hapticData, subPacketTypeHaptic);
        }
    }
}

static inline uint8_t* __not_in_flash_func(getSubPacketBufferHaptic)() {
    uint8_t* ret = tryGetSubPacketBufferHaptic();
    if (ret == nullptr) {
        cleanAllCachedHaptic();
        ret = tryGetSubPacketBufferHaptic();
    }

    if (ret == nullptr) {
        LOGE("No free subPacketBufferHaptic");
    }
    return ret;
}

static inline uint8_t* __not_in_flash_func(tryGetSubPacketBufferControl)() {
    for (int i = 0; i < subPacketBuffControlCount; i++) {
        if (!bluetoothPacket.subPacketBufferControl[i].inuse) {
            bluetoothPacket.subPacketBufferControl[i].inuse = true;
            return bluetoothPacket.subPacketBufferControl[i].buf;
        }
    }

    return nullptr;
}

static inline uint8_t* __not_in_flash_func(getSubPacketBufferControl)() {
    uint8_t* ret = tryGetSubPacketBufferControl();
    if (ret == nullptr) {
        // control 只需要有最后一次的就行
        uint8_t* controlData = nullptr;
        while (queue_try_remove(&bluetoothPacket.subPacketControlQueue, &controlData)) {
            if (controlData != nullptr) {
                freeSubPacket(controlData, subPacketTypeControl);
            }
        }

        ret = tryGetSubPacketBufferControl();
    }

    if (ret == nullptr) {
        LOGE("No free subPacketBufferControl");
    }
    return ret;
}

static inline uint8_t* __not_in_flash_func(tryGetSubPacketBufferAudio)() {
    for (int i = 0; i < subPacketBuffAudioCount; i++) {
        bool expected = false;
        if (atomic_compare_exchange_strong(&bluetoothPacket.subPacketBufferAudio[i].inuse, &expected, true)) {
            return bluetoothPacket.subPacketBufferAudio[i].buf;
        }
    }

    return nullptr;
}

void cleanAllCachedAudio() {
    uint8_t* audioData = nullptr;
    while (queue_try_remove(&bluetoothPacket.subPacketAudioQueue, &audioData)) {
        if (audioData != nullptr) {
            freeSubPacket(audioData, subPacketTypeAudio);
        }
    }
}

static inline uint8_t* __not_in_flash_func(getSubPacketBufferAudio)() {
    uint8_t* ret = tryGetSubPacketBufferAudio();
    if (ret == nullptr) {
        cleanAllCachedAudio();
        ret = tryGetSubPacketBufferAudio();
    }

    if (ret == nullptr) {
        LOGE("No free subPacketBufferAudio");
    }
    return ret;
}

void bluetoothPacketInit() {
    for (int i = 0; i < subPacketBuffHapticCount; i++) {
        bluetoothPacket.subPacketBufferHaptic[i].inuse = false;
    }
    for (int i = 0; i < subPacketBuffControlCount; i++) {
        bluetoothPacket.subPacketBufferControl[i].inuse = false;
    }
    for (int i = 0; i < subPacketBuffAudioCount; i++) {
        atomic_init(&bluetoothPacket.subPacketBufferAudio[i].inuse, false);
    }
    for (int i = 0; i < bluetoothRawPacketCount; i++) {
        bluetoothPacket.bluetoothRawPacket[i].size = 0;
        bluetoothPacket.bluetoothRawPacket[i].inuse = false;
    }

    bluetoothPacket.reportSeqCounter = 0;
    bluetoothPacket.packetCounter = 0;
    bluetoothPacket.needSendAudioSetup = false;
    bluetoothPacket.needSendControl = false;

    queue_init(&bluetoothPacket.subPacketHapticQueue, sizeof(uint8_t*), subPacketBuffHapticCount);
    queue_init(&bluetoothPacket.subPacketControlQueue, sizeof(uint8_t*), subPacketBuffControlCount);
    queue_init(&bluetoothPacket.subPacketAudioQueue, sizeof(uint8_t*), subPacketBuffAudioCount);
}

uint8_t* __not_in_flash_func(getBufferForSubPacket)(const enum subPacketType type) {
    switch (type) {
        case subPacketTypeControl:
            return getSubPacketBufferControl();
        case subPacketTypeHaptic:
            return getSubPacketBufferHaptic();
        case subPacketTypeAudio:
            return getSubPacketBufferAudio();
        default:
            LOGE("error subPacketType:%d", type);
            return nullptr;
    }
}

void __not_in_flash_func(freeSubPacket)(uint8_t* buffer, const enum subPacketType type) {
    if (buffer == nullptr) {
        return;
    }

    switch (type) {
        case subPacketTypeControl: {
            struct SubPacketBufferControl* pkt = container_of(buffer, struct SubPacketBufferControl, buf);
            pkt->inuse = false;
            break;
        }
        case subPacketTypeHaptic: {
            struct SubPacketBufferHaptic* pkt = container_of(buffer, struct SubPacketBufferHaptic, buf);
            pkt->inuse = false;
            break;
        }
        case subPacketTypeAudio: {
            struct SubPacketBufferAudio* pkt = container_of(buffer, struct SubPacketBufferAudio, buf);
            atomic_store(&pkt->inuse, false);
            break;
        }
        default:
            LOGE("error subPacketType:%d", type);
    }
}

void __not_in_flash_func(writeSubPacket)(uint8_t* buffer, const enum subPacketType type) {
    switch (type) {
        case subPacketTypeControl: {
            if (!queue_try_add(&bluetoothPacket.subPacketControlQueue, (void*)&buffer)) {
                LOGW("subPacketControl add failed");
                freeSubPacket(buffer, type);
                return;
            }
            const union Ds5ControlUnion* controlUnion = (union Ds5ControlUnion*)buffer;
            bluetoothPacket.needSendControl = controlUnion->packet.UseRumbleNotHaptics != 0 || controlUnion->packet.UseRumbleNotHaptics2 != 0;
            break;
        }
        case subPacketTypeHaptic: {
            if (!queue_try_add(&bluetoothPacket.subPacketHapticQueue, (void*)&buffer)) {
                LOGW("subPacketHaptic add failed");
                freeSubPacket(buffer, type);
                return;
            }
            break;
        }
        case subPacketTypeAudio: {
            if (!queue_try_add(&bluetoothPacket.subPacketAudioQueue, (void*)&buffer)) {
                LOGW("subPacketAudio add failed");
                freeSubPacket(buffer, type);
                return;
            }
            break;
        }
        default:
            LOGE("error subPacketType:%d", type);
            return;
    }

    // audio 在另一个core运行，直接调用会有问题
    if (type != subPacketTypeAudio) {
        btRequestSend();
    }
}

void __not_in_flash_func(freeBluetoothRawPacket)(uint8_t* bluetoothRawPacket) {
    if (bluetoothRawPacket == nullptr) {
        LOGE("bluetoothRawPacket is nullptr");
        return;
    }

    struct BluetoothRawPacket* realPkt = container_of(bluetoothRawPacket, struct BluetoothRawPacket, data);
    realPkt->inuse = false;
}

static inline struct BluetoothRawPacket* __not_in_flash_func(newBluetoothRawPacket)(const size_t size) {
    size_t dataSize = 0;
    uint8_t reportId = 0;
    for (int i = 0; i < (sizeof(bluetoothRawPacketReportIDAndDataSizeTrack) / sizeof(bluetoothRawPacketReportIDAndDataSizeTrack[0])); i++) {
        if (size <= bluetoothRawPacketReportIDAndDataSizeTrack[i].dataSize) {
            dataSize = bluetoothRawPacketReportIDAndDataSizeTrack[i].dataSize;
            reportId = bluetoothRawPacketReportIDAndDataSizeTrack[i].reportId;
            break;
        }
    }

    if (dataSize == 0 || reportId == 0) {
        LOGE("packet too large, no valid report id for size:%d", size);
        return nullptr;
    }

    struct BluetoothRawPacket* freePkt = nullptr;
    for (int i = 0; i < bluetoothRawPacketCount; i++) {
        if (!bluetoothPacket.bluetoothRawPacket[i].inuse) {
            bluetoothPacket.bluetoothRawPacket[i].inuse = true;
            freePkt = &bluetoothPacket.bluetoothRawPacket[i];
            break;
        }
    }

    if (freePkt == nullptr) {
        LOGE("no free bluetoothPacket");
        return nullptr;
    }

    freePkt->size = dataSize;
    freePkt->data[0] = 0XA2;
    freePkt->data[1] = reportId;
    freePkt->data[2] = bluetoothPacket.reportSeqCounter << 4;
    bluetoothPacket.reportSeqCounter = (bluetoothPacket.reportSeqCounter + 1) & 0x0F;

    return freePkt;
}

bool __not_in_flash_func(hasBluetoothRawPacketCanSend)() {
    if (bluetoothPacket.needSendControl) {
        return true;
    }

    const uint hapticCount = queue_get_level(&bluetoothPacket.subPacketHapticQueue);
    const uint audioCount = queue_get_level(&bluetoothPacket.subPacketAudioQueue);

    // 有音频数据的时候就把control放到音频数据中发送，防止发送过于频繁
    if (config.audioActive) {
        // 最好的情况是audio和haptic一起发送
        if (hapticCount > 0 && audioCount > 0) {
            return true;
        }

        // 如果单独某一种包积累太多就直接发送
        if (hapticCount >= 2 || audioCount >= 2) {
            return true;
        }

        return false;
    }

    if (bluetoothPacket.needSendAudioSetup) {
        return true;
    }

    const uint controlCount = queue_get_level(&bluetoothPacket.subPacketControlQueue);
    return (hapticCount + controlCount + audioCount) > 0;
}

// Sub-Packet  0x11: AUDIO_SETUP（音频配置帧） （共 9 字节）
/*
[0x91]  [0x07]  [0xFE]  [0x00]  [0x00]  [0x00]  [0x00]  [0x48]  [counter]
 PID     len    flags    ???     ???     ???     ???     cache  frame_no
*/
// DUALSENSE_BT_REPORT_AUDIO_SETUP = 0x11, // AUDIO_SETUP（音频配置帧）
// DUALSENSE_BT_REPORT_MASK_HAS_LENGTH = 1 << 7,  // 0x80 — 修饰位：有 length 字段
// DUALSENSE_BT_REPORT_MASK_HAS_DOUBLE = 1 << 6,  // 0x40 — 修饰位：双通道扩展 (默认不设置)
// 设置 PID = 0x11，并用 0x80 标记有 length 字段
static inline int __not_in_flash_func(setAudioSetupSubPacket)(uint8_t* buffer) {
    // sub packet head
    buffer[0] = (0x11 | 1 << 7) & (~(1 << 6));
    buffer[1] = subPacketAudioSetupSize;
    // sub packet content

    buffer[2] = (config.micActive && !config.disableMic) ? 0b11111111 : 0b11111110;  // AudioFlags: 启用全部音频路由
    buffer[3] = 0;                                                                   // 保留
    buffer[4] = 0;                                                                   // 保留
    buffer[5] = 0;                                                                   // 保留
    buffer[6] = 0;                                                                   // 保留
    buffer[7] = config.audioBufferLength;                                            // 可能是缓存大小，影响延迟
    buffer[8] = bluetoothPacket.packetCounter++;                                     // 帧计数器，单调递增

    static_assert(subPacketHeadSize + subPacketAudioSetupSize == 9, "audio setup sub packet size should be 9");
    return subPacketHeadSize + subPacketAudioSetupSize;
}

// Sub-Packet  0x12: HAPTICS_DATA（共 66 字节）
/*
[0x92]  [0x40]  [PCM data × 64 bytes ...]
 PID     len=64  int8_t stereo samples @ 3kHz
*/
// HAPTICS_DATA = 0x12, // HAPTICS数据 PCM int8_t stereo samples @ 3kHz
// DUALSENSE_BT_REPORT_MASK_HAS_LENGTH = 1 << 7,  // 0x80 — 修饰位：有 length 字段
// DUALSENSE_BT_REPORT_MASK_HAS_DOUBLE = 1 << 6,  // 0x40 — 修饰位：双通道扩展 (默认不设置)
static inline int __not_in_flash_func(setHapticSubPacket)(uint8_t* buffer, const uint8_t* hapticData) {
    // sub packet head
    buffer[0] = (0x12 | 1 << 7) & (~(1 << 6));
    buffer[1] = subPacketHapticSize;
    // sub packet content
    memcpy(buffer + subPacketHeadSize, hapticData, subPacketHapticSize);

    static_assert(subPacketHeadSize + subPacketHapticSize == 66, "haptic sub packet size should be 66");
    return subPacketHeadSize + subPacketHapticSize;
}

// Sub-packet 0x10 : control
/*
[0x90]      [0x3F]    [control data × 63 bytes ...]
 PID        len=63   control data
// STATUS_DATA = 0x12, // STATUS数据 63字节
// DUALSENSE_BT_REPORT_MASK_HAS_LENGTH = 1 << 7,  // 0x80 — 修饰位：有 length 字段
// DUALSENSE_BT_REPORT_MASK_HAS_DOUBLE = 1 << 6,  // 0x40 — 修饰位：双通道扩展 (默认不设置)
*/
static inline int __not_in_flash_func(setControlSubPacket)(uint8_t* buffer, const uint8_t* controlData) {
    // sub packet head
    buffer[0] = (0x10 | 1 << 7) & (~(1 << 6));
    buffer[1] = subPacketControlSize;
    // sub packet content
    memcpy(buffer + subPacketHeadSize, controlData, subPacketControlSize);

    static_assert(subPacketHeadSize + subPacketControlSize == 65, "control sub packet size should be 65");
    return subPacketHeadSize + subPacketControlSize;
}

// Sub-Packet Speaker: 0x13,  L Headset Mono: 0x14, L Headset R Speaker: 0x15, Headset: 0x16 (共 202 字节)
/*
[0x??]  [0xc8]  [opus data × 200 bytes ...]
 PID     len=200  opus data
*/
// Speaker: 0x13,L Headset Mono: 0x14,L Headset R Speaker: 0x15, Headset: 0x16 // 声音数据 opus编码
// DUALSENSE_BT_REPORT_MASK_HAS_LENGTH = 1 << 7,  // 0x80 — 修饰位：有 length 字段
// DUALSENSE_BT_REPORT_MASK_HAS_DOUBLE = 1 << 6,  // 0x40 — 修饰位：双通道扩展
static inline int __not_in_flash_func(setAudioSubPacket)(uint8_t* buffer, const uint8_t* audioData) {
    // sub packet head
    buffer[0] = config.plugHeadset ? (0x16 | 1 << 7) & (~(1 << 6)) : (0x13 | 1 << 7) & (~(1 << 6));
    buffer[1] = subPacketAudioSize;
    // sub packet content
    memcpy(buffer + subPacketHeadSize, audioData, subPacketAudioSize);

    static_assert(subPacketHeadSize + subPacketAudioSize == 202, "audio sub packet size should be 202");
    return subPacketHeadSize + subPacketAudioSize;
}

static inline void __not_in_flash_func(packed)(const uint8_t* controlData, uint8_t** hapticData, uint8_t** audioData, struct BluetoothRawPacket* pkt) {
    size_t offset = bluetoothRawPacketHeadSize + ds5BluetoothPacketHeadSize;

    if (controlData != nullptr) {
        if (bluetoothPacket.needSendControl) {
            bluetoothPacket.needSendControl = false;
        }
        offset += setControlSubPacket(pkt->data + offset, controlData);
    }

    bool addedAudioSetup = false;
    for (int i = 0; i < 2; i++) {
        if (hapticData[i] != nullptr) {
            if (!addedAudioSetup) {
                offset += setAudioSetupSubPacket(pkt->data + offset);
                addedAudioSetup = true;
            }

            offset += setHapticSubPacket(pkt->data + offset, hapticData[i]);
        }
    }

    for (int i = 0; i < 2; i++) {
        if (audioData[i] != nullptr) {
            if (!addedAudioSetup) {
                offset += setAudioSetupSubPacket(pkt->data + offset);
                addedAudioSetup = true;
            }

            offset += setAudioSubPacket(pkt->data + offset, audioData[i]);
        }
    }

    if (bluetoothPacket.needSendAudioSetup) {
        bluetoothPacket.needSendAudioSetup = false;
        if (!addedAudioSetup) {
            offset += setAudioSetupSubPacket(pkt->data + offset);
            addedAudioSetup = true;
        }
    }

    if (offset + ds5BluetoothPacketCrc32Size < pkt->size) {
        memset(pkt->data + offset, 0, pkt->size - offset - ds5BluetoothPacketCrc32Size);
    } else if (offset + ds5BluetoothPacketCrc32Size > pkt->size) {
        LOGE("packet size is too small:offset + ds5BluetoothPacketCrc32Size:%d, pkt->size:%d, pkt->data[1]:%02X", offset + ds5BluetoothPacketCrc32Size, pkt->size, pkt->data[1]);
    }

    fillOutputReportChecksum(pkt->data + bluetoothRawPacketHeadSize, pkt->size - bluetoothRawPacketHeadSize);
}

uint8_t* __not_in_flash_func(getBluetoothRawPacket)(size_t* size) {
    uint8_t* hapticData[2] = {nullptr, nullptr};
    uint8_t* audioData[2] = {nullptr, nullptr};
    uint8_t* controlData = nullptr;
    size_t pktSize = 0;
    bool haveAudioSetup = false;

    // 优先尝试拿两个hapticData
    queue_try_remove(&bluetoothPacket.subPacketHapticQueue, &hapticData[0]);
    queue_try_remove(&bluetoothPacket.subPacketHapticQueue, &hapticData[1]);
    for (int i = 0; i < 2; i++) {
        if (hapticData[i] != nullptr) {
            pktSize += subPacketHeadSize;
            pktSize += subPacketHapticSize;
            // 66
        }
    }

    queue_try_remove(&bluetoothPacket.subPacketAudioQueue, &audioData[0]);
    // 如果没有hapticData，就尝试拿两个audioData
    if (pktSize == 0) {
        queue_try_remove(&bluetoothPacket.subPacketAudioQueue, &audioData[1]);
    }

    for (int i = 0; i < 2; i++) {
        if (audioData[i] != nullptr) {
            pktSize += subPacketHeadSize;
            pktSize += subPacketAudioSize;
            // 202
        }
    }

    // 只要有 audioData 或者 hapticData 就要加上 audioSetup
    if (pktSize > 0) {
        haveAudioSetup = true;
        pktSize += subPacketHeadSize;
        pktSize += subPacketAudioSetupSize;
        // 9
    }

    // audioActive时把controlData一起发送，防止发送频繁
    if (config.audioActive && pktSize == 0 && !bluetoothPacket.needSendControl) {
        // 没有hapticData 没有audioData
        return nullptr;
    }

    // 尝试拿controlData
    queue_try_remove(&bluetoothPacket.subPacketControlQueue, &controlData);
    if (controlData != nullptr) {
        pktSize += subPacketHeadSize;
        pktSize += subPacketControlSize;
        // 65
    }

    if (bluetoothPacket.needSendAudioSetup && !haveAudioSetup) {
        pktSize += subPacketHeadSize;
        pktSize += subPacketAudioSetupSize;
        // 9
    }

    // 没有数据包
    if (pktSize == 0) {
        return nullptr;
    }

    pktSize += bluetoothRawPacketHeadSize;
    pktSize += ds5BluetoothPacketHeadSize;
    pktSize += ds5BluetoothPacketCrc32Size;
    // 7

    // 可以判断一下是否还能再装一个audioData (当只有一个hapticData + 一个audioData + 没有controlData的时候，剩余空间是够的)
    if ((bluetoothRawPacketDataSize0x39 - pktSize) > (subPacketHeadSize + subPacketAudioSize)) {
        if (audioData[0] == nullptr && audioData[1] != nullptr) {
            audioData[0] = audioData[1];
            audioData[1] = nullptr;
        }

        if (audioData[1] == nullptr) {
            queue_try_remove(&bluetoothPacket.subPacketAudioQueue, &audioData[1]);
            if (audioData[1] != nullptr) {
                pktSize += subPacketHeadSize;
                pktSize += subPacketAudioSize;
                // 202

                if (!haveAudioSetup) {
                    haveAudioSetup = true;
                    pktSize += subPacketHeadSize;
                    pktSize += subPacketAudioSetupSize;
                    // 9
                }
            }
        }
    }

    // 最大size能够支持 bluetoothRawPacketDataSize0x39 -> 548
    // 最大的情况: 1. 两个音频包，那么就要少一个hapticData或者少一个controlData
    //           2. 1个音频包两个haptic包

    struct BluetoothRawPacket* pkt = newBluetoothRawPacket(pktSize);
    if (pkt == nullptr) {
        freeSubPacket(hapticData[0], subPacketTypeHaptic);
        freeSubPacket(hapticData[1], subPacketTypeHaptic);
        freeSubPacket(audioData[0], subPacketTypeAudio);
        freeSubPacket(audioData[1], subPacketTypeAudio);
        freeSubPacket(controlData, subPacketTypeControl);
        return nullptr;
    }

    packed(controlData, hapticData, audioData, pkt);
    freeSubPacket(hapticData[0], subPacketTypeHaptic);
    freeSubPacket(hapticData[1], subPacketTypeHaptic);
    freeSubPacket(audioData[0], subPacketTypeAudio);
    freeSubPacket(audioData[1], subPacketTypeAudio);
    freeSubPacket(controlData, subPacketTypeControl);

    *size = pkt->size;
    return pkt->data;
}
