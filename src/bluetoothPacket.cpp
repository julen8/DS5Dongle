#include "bluetoothPacket.h"

#include <pico/util/queue.h>

#include <cstdint>
#include <cstdlib>
#include <cstring>

#include "config.h"
#include "log.h"
#include "utils.h"

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
subpacket content    (HapticSetup:7 , Haptic:64, status:63, Audio:200)
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
│  │  │ Sub-Packet #1: HAPTICS_SETUP (0x91)，9字节   │   │  │
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

constexpr auto bluetoothRawPacketHeadSize = 1;
constexpr auto ds5BluetoothPacketHeadSize = 2;

constexpr auto subPacketHeadSize = 2;
constexpr auto subPacketHapticSetupSize = 7;

constexpr auto ds5BluetoothPacketCrc32Size = 4;

constexpr auto subPacketBuffHapticCount = 4;
constexpr auto subPacketBuffStatusCount = 4;
constexpr auto subPacketBuffAudioCount = 4;
constexpr auto bluetoothRawPacketCount = 4;

extern "C" {
#ifndef offsetof
#    define offsetof(TYPE, MEMBER) ((size_t)&((TYPE*)0)->MEMBER)
#endif

#define container_of(ptr, type, member) ({ (type*)((char*)(ptr) - offsetof(type, member)); })

struct SubPacketBufferHaptic {
    bool inuse;
    uint8_t buf[subPacketHapticSize];
};
struct SubPacketBufferStatus {
    bool inuse;
    uint8_t buf[subPacketStatusSize];
};
struct SubPacketBufferAudio {
    bool inuse;
    uint8_t buf[subPacketAudioSize];
};

/*
https://controllers.fandom.com/wiki/Sony_DualSense
REPORT_ID
eportID[1]	Size	Type	Note
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
struct BluetoothRawPacket {
    size_t size;
    bool inuse;
    uint8_t data[399];  // 包含bluetoothRawPacketHeadSize和REPORT_ID所以比上面看到的值+2
};

static struct {
    SubPacketBufferHaptic subPacketBufferHaptic[subPacketBuffHapticCount];
    SubPacketBufferStatus subPacketBufferStatus[subPacketBuffStatusCount];
    SubPacketBufferAudio subPacketBufferAudio[subPacketBuffAudioCount];
    BluetoothRawPacket bluetoothRawPacket[bluetoothRawPacketCount];

    queue_t subPacketHapticQueue;
    queue_t subPacketStatusQueue;
    queue_t subPacketAudioQueue;

    onWriteCallbackType onWriteCallback = nullptr;
    uint8_t reportSeqCounter = 0;
    uint8_t packetCounter = 0;

} bluetoothPacket{};
}

inline uint8_t* getSubPacketBufferHaptic() {
    for (auto& elem : bluetoothPacket.subPacketBufferHaptic) {
        if (!elem.inuse) {
            elem.inuse = true;
            LOGD("haptic pkt:%p", &elem);
            return elem.buf;
        }
    }

    LOGE("No free subPacketBufferHaptic");
    return nullptr;
}

inline uint8_t* tryGetSubPacketBufferStatus() {
    for (auto& elem : bluetoothPacket.subPacketBufferStatus) {
        if (!elem.inuse) {
            elem.inuse = true;
            return elem.buf;
        }
    }

    return nullptr;
}

inline uint8_t* getSubPacketBufferStatus() {
    if (config.audioActive) {
        return bluetoothPacket.subPacketBufferStatus[0].buf;
    }

    auto* ret = tryGetSubPacketBufferStatus();
    if (ret == nullptr) {
        // status 只需要有最后一次的就行
        uint8_t* statusData = nullptr;
        queue_try_remove(&bluetoothPacket.subPacketStatusQueue, static_cast<void*>(&statusData));
        if (statusData != nullptr) {
            freeSubPacket(statusData, subPacketType::status);
        }
        ret = tryGetSubPacketBufferStatus();
    }

    if (ret == nullptr) {
        LOGE("No free subPacketBufferStatus");
    }
    return ret;
}

inline uint8_t* getSubPacketBufferAudio() {
    for (auto& elem : bluetoothPacket.subPacketBufferAudio) {
        if (!elem.inuse) {
            elem.inuse = true;
            return elem.buf;
        }
    }

    LOGE("No free subPacketBufferAudio");
    return nullptr;
}

void bluetoothPacketInit() {
    for (auto& elem : bluetoothPacket.subPacketBufferHaptic) {
        elem.inuse = false;
    }
    for (auto& elem : bluetoothPacket.subPacketBufferStatus) {
        elem.inuse = false;
    }
    for (auto& elem : bluetoothPacket.subPacketBufferAudio) {
        elem.inuse = false;
    }
    for (auto& elem : bluetoothPacket.bluetoothRawPacket) {
        elem.size = 0;
        elem.inuse = false;
    }

    queue_init(&bluetoothPacket.subPacketHapticQueue, sizeof(uint8_t*), subPacketBuffHapticCount);
    queue_init(&bluetoothPacket.subPacketStatusQueue, sizeof(uint8_t*), subPacketBuffStatusCount);
    queue_init(&bluetoothPacket.subPacketAudioQueue, sizeof(uint8_t*), subPacketBuffAudioCount);

    memcpy(bluetoothPacket.subPacketBufferStatus[0].buf, stateInitData, subPacketStatusSize);
}

uint8_t* getBufferForSubPacket(const subPacketType type) {
    switch (type) {
        case subPacketType::status:
            return getSubPacketBufferStatus();
        case subPacketType::haptic:
            return getSubPacketBufferHaptic();
        case subPacketType::audio:
            return getSubPacketBufferAudio();
        default:
            LOGE("error subPacketType:%d", type);
            return nullptr;
    }
}

void freeSubPacket(uint8_t* buffer, const subPacketType type) {
    if (buffer == nullptr) {
        return;
    }

    switch (type) {
        case subPacketType::status: {
            if (config.audioActive) {
                return;
            }
            auto* pkt = container_of(buffer, SubPacketBufferStatus, buf);
            pkt->inuse = false;
            break;
        }
        case subPacketType::haptic: {
            auto* pkt = container_of(buffer, SubPacketBufferHaptic, buf);
            LOGD("haptic pkt:%p", pkt);
            pkt->inuse = false;
            break;
        }
        case subPacketType::audio: {
            auto* pkt = container_of(buffer, SubPacketBufferAudio, buf);
            pkt->inuse = false;
            break;
        }
        default:
            LOGE("error subPacketType:%d", type);
    }
}

void writeSubPacket(uint8_t* buffer, const subPacketType type) {
    LOGD("type:%d", type);
    switch (type) {
        case subPacketType::status: {
            if (config.audioActive) {
                return;
            }
            if (!queue_try_add(&bluetoothPacket.subPacketStatusQueue, static_cast<void*>(&buffer))) {
                LOGW("subPacketStatus add failed");
                freeSubPacket(buffer, type);
            }
            break;
        }
        case subPacketType::haptic: {
            if (!queue_try_add(&bluetoothPacket.subPacketHapticQueue, static_cast<void*>(&buffer))) {
                LOGW("subPacketStatus add failed");
                freeSubPacket(buffer, type);
            }
            break;
        }
        case subPacketType::audio: {
            if (!queue_try_add(&bluetoothPacket.subPacketAudioQueue, static_cast<void*>(&buffer))) {
                LOGW("subPacketStatus add failed");
                freeSubPacket(buffer, type);
            }
            break;
        }
        default:
            LOGE("error subPacketType:%d", type);
            return;
    }

    // audio 在另一个core运行，直接调用会有问题
    if (bluetoothPacket.onWriteCallback != nullptr && type != subPacketType::audio) {
        bluetoothPacket.onWriteCallback();
    }
}

void freeBluetoothRawPacket(uint8_t* bluetoothRawPacket) {
    if (bluetoothRawPacket == nullptr) {
        LOGE("bluetoothRawPacket is nullptr");
        return;
    }

    auto* realPkt = container_of(bluetoothRawPacket, BluetoothRawPacket, data);
    realPkt->inuse = false;
}

inline BluetoothRawPacket* newBluetoothRawPacket(const int size) {
    BluetoothRawPacket* freePkt = nullptr;
    for (auto& elem : bluetoothPacket.bluetoothRawPacket) {
        if (!elem.inuse) {
            elem.inuse = true;
            freePkt = &elem;
            break;
        }
    }

    if (freePkt == nullptr) {
        LOGE("no free bluetoothPacket");
        return nullptr;
    }

    if (size <= 79) {
        freePkt->size = 79;
        freePkt->data[1] = 0x31;
    } else if (size < 143) {
        freePkt->size = 143;
        freePkt->data[1] = 0x32;
    } else if (size < 207) {
        freePkt->size = 207;
        freePkt->data[1] = 0x33;
    } else if (size < 271) {
        freePkt->size = 271;
        freePkt->data[1] = 0x34;
    } else if (size <= 335) {
        freePkt->size = 335;
        freePkt->data[1] = 0x35;
    } else if (size <= 399) {
        freePkt->size = 399;
        freePkt->data[1] = 0x36;
    } else {
        freeBluetoothRawPacket(freePkt->data);
        LOGE("packet too large, no valid report id for size:%d", size);
        return nullptr;
    }

    freePkt->data[0] = 0XA2;
    freePkt->data[2] = bluetoothPacket.reportSeqCounter << 4;
    bluetoothPacket.reportSeqCounter = (bluetoothPacket.reportSeqCounter + 1) & 0x0F;

    return freePkt;
}

size_t getSubPacketSize() {
    const auto hapticCount = queue_get_level(&bluetoothPacket.subPacketHapticQueue);
    const auto audioCount = queue_get_level(&bluetoothPacket.subPacketAudioQueue);

    // 有音频数据的时候就把status放到音频数据中发送，防止发送过于频繁
    if (config.audioActive) {
        // 最好的情况是audio和haptic一起发送
        if (hapticCount > 0 && audioCount > 0) {
            return hapticCount;
        }

        // 如果积累太多就直接发送
        if (hapticCount >= 2) {
            return hapticCount;
        }
        if (audioCount >= 2) {
            return audioCount;
        }

        return 0;
    }

    const auto statusCount = queue_get_level(&bluetoothPacket.subPacketStatusQueue);
    return hapticCount + statusCount + audioCount;
}

void setBluetoothSubPacketWriteCallback(const onWriteCallbackType callback) { bluetoothPacket.onWriteCallback = callback; }

// Sub-Packet  0x11: HAPTICS_SETUP（音频配置帧） （共 9 字节）
/*
[0x91]  [0x07]  [0xFE]  [0x00]  [0x00]  [0x00]  [0x00]  [0x48]  [counter]
 PID     len    flags    ???     ???     ???     ???     cache  frame_no
*/
// DUALSENSE_BT_REPORT_HAPTICS_SETUP = 0x11, // HAPTICS_SETUP（音频配置帧）
// DUALSENSE_BT_REPORT_MASK_HAS_LENGTH = 1 << 7,  // 0x80 — 修饰位：有 length 字段
// DUALSENSE_BT_REPORT_MASK_HAS_DOUBLE = 1 << 6,  // 0x40 — 修饰位：双通道扩展 (默认不设置)
// 设置 PID = 0x11，并用 0x80 标记有 length 字段
inline int setHapticSetupSubPacket(uint8_t* buffer) {
    // sub packet head
    buffer[0] = (0x11 | 1 << 7) & (~(1 << 6));
    buffer[1] = subPacketHapticSetupSize;
    // sub packet content
    buffer[2] = 0b11111110;                       // AudioFlags: 启用全部音频路由
    buffer[3] = 0;                                // 保留
    buffer[4] = 0;                                // 保留
    buffer[5] = 0;                                // 保留
    buffer[6] = 0;                                // 保留
    buffer[7] = config.audioBufferLength;         // 可能是缓存大小，影响延迟
    buffer[8] = bluetoothPacket.packetCounter++;  // 帧计数器，单调递增

    static_assert(subPacketHeadSize + subPacketHapticSetupSize == 9, "haptic setup sub packet size should be 9");
    return subPacketHeadSize + subPacketHapticSetupSize;
}

// Sub-Packet  0x12: HAPTICS_DATA（共 66 字节）
/*
[0x92]  [0x40]  [PCM data × 64 bytes ...]
 PID     len=64  int8_t stereo samples @ 3kHz
*/
// HAPTICS_DATA = 0x12, // HAPTICS数据 PCM int8_t stereo samples @ 3kHz
// DUALSENSE_BT_REPORT_MASK_HAS_LENGTH = 1 << 7,  // 0x80 — 修饰位：有 length 字段
// DUALSENSE_BT_REPORT_MASK_HAS_DOUBLE = 1 << 6,  // 0x40 — 修饰位：双通道扩展 (默认不设置)
inline int setHapticSubPacket(uint8_t* buffer, const uint8_t* hapticData) {
    // sub packet head
    buffer[0] = (0x12 | 1 << 7) & (~(1 << 6));
    buffer[1] = subPacketHapticSize;
    // sub packet content
    memcpy(buffer + subPacketHeadSize, hapticData, subPacketHapticSize);

    static_assert(subPacketHeadSize + subPacketHapticSize == 66, "haptic sub packet size should be 66");
    return subPacketHeadSize + subPacketHapticSize;
}

// Sub-packet 0x10 : status
/*
[0x90]      [0x3F]    [status data × 63 bytes ...]
 PID        len=63   status data
// STATUS_DATA = 0x12, // STATUS数据 63字节
// DUALSENSE_BT_REPORT_MASK_HAS_LENGTH = 1 << 7,  // 0x80 — 修饰位：有 length 字段
// DUALSENSE_BT_REPORT_MASK_HAS_DOUBLE = 1 << 6,  // 0x40 — 修饰位：双通道扩展 (默认不设置)
*/
inline int setStatusSubPacket(uint8_t* buffer, const uint8_t* statusData) {
    // sub packet head
    buffer[0] = (0x10 | 1 << 7) & (~(1 << 6));
    buffer[1] = subPacketStatusSize;
    // sub packet content
    memcpy(buffer + subPacketHeadSize, statusData, subPacketStatusSize);

    static_assert(subPacketHeadSize + subPacketStatusSize == 65, "status sub packet size should be 65");
    return subPacketHeadSize + subPacketStatusSize;
}

// Sub-Packet Speaker: 0x13,  L Headset Mono: 0x14, L Headset R Speaker: 0x15, Headset: 0x16 (共 202 字节)
/*
[0x??]  [0xc8]  [opus data × 200 bytes ...]
 PID     len=200  opus data
*/
// Speaker: 0x13,L Headset Mono: 0x14,L Headset R Speaker: 0x15, Headset: 0x16 // 声音数据 opus编码
// DUALSENSE_BT_REPORT_MASK_HAS_LENGTH = 1 << 7,  // 0x80 — 修饰位：有 length 字段
// DUALSENSE_BT_REPORT_MASK_HAS_DOUBLE = 1 << 6,  // 0x40 — 修饰位：双通道扩展
inline int setAudioSubPacket(uint8_t* buffer, const uint8_t* audioData) {
    // sub packet head
    buffer[0] = config.plugHeadset ? (0x16 | 1 << 7) & (~(1 << 6)) : (0x13 | 1 << 7) & (~(1 << 6));
    buffer[1] = subPacketAudioSize;
    // sub packet content
    memcpy(buffer + subPacketHeadSize, audioData, subPacketAudioSize);

    static_assert(subPacketHeadSize + subPacketAudioSize == 202, "audio sub packet size should be 202");
    return subPacketHeadSize + subPacketAudioSize;
}

inline BluetoothRawPacket* packed(const uint8_t* hapticData, const uint8_t* statusData, const uint8_t* audioData, BluetoothRawPacket* pkt) {
    size_t offset = bluetoothRawPacketHeadSize + ds5BluetoothPacketHeadSize;

    if (statusData != nullptr) {
        offset += setStatusSubPacket(pkt->data + offset, statusData);
    }

    if (hapticData != nullptr || audioData != nullptr) {
        offset += setHapticSetupSubPacket(pkt->data + offset);

        if (hapticData != nullptr) {
            offset += setHapticSubPacket(pkt->data + offset, hapticData);
        }

        if (audioData != nullptr) {
            offset += setAudioSubPacket(pkt->data + offset, audioData);
        }
    }

    if (offset + ds5BluetoothPacketCrc32Size < pkt->size) {
        memset(pkt->data + offset, 0, pkt->size - offset - ds5BluetoothPacketCrc32Size);
    } else if (offset + ds5BluetoothPacketCrc32Size > pkt->size) {
        LOGE("packet size is too small:offset + ds5BluetoothPacketCrc32Size:%d, pkt->size:%d", offset + ds5BluetoothPacketCrc32Size, pkt->size);
    }

    fill_output_report_checksum(pkt->data + bluetoothRawPacketHeadSize, pkt->size - bluetoothRawPacketHeadSize);
    return pkt;
}

uint8_t* getBluetoothRawPacket(size_t* size) {
    uint8_t* hapticData = nullptr;
    uint8_t* statusData = nullptr;
    uint8_t* audioData = nullptr;

    if (config.audioActive) {
        statusData = bluetoothPacket.subPacketBufferStatus[0].buf;
    } else {
        queue_try_remove(&bluetoothPacket.subPacketStatusQueue, static_cast<void*>(&statusData));
    }
    queue_try_remove(&bluetoothPacket.subPacketHapticQueue, static_cast<void*>(&hapticData));
    queue_try_remove(&bluetoothPacket.subPacketAudioQueue, static_cast<void*>(&audioData));

    auto pktSize = 0;
    if (hapticData != nullptr) {
        pktSize += subPacketHeadSize;
        pktSize += subPacketHapticSetupSize;

        pktSize += subPacketHeadSize;
        pktSize += subPacketHapticSize;
        // 75
    }

    if (statusData != nullptr) {
        pktSize += subPacketHeadSize;
        pktSize += subPacketStatusSize;
        // 66
    }

    if (audioData != nullptr) {
        pktSize += subPacketHeadSize;
        pktSize += subPacketAudioSize;
        // 202
    }

    if (pktSize == 0) {
        return nullptr;
    }

    pktSize += bluetoothRawPacketHeadSize;
    pktSize += ds5BluetoothPacketHeadSize;
    pktSize += ds5BluetoothPacketCrc32Size;
    // 7

    // 可能的size：75+7 / 66+7 / 202+7 / 75+66+7 / 75+202+7 / 66+202+7 / 75+66+202+7
    //             82      73    209      148        284    275           350
    // 实际上大部分情况下haptic和audio总是同时有或者没有(当!config.audioActive的时候还是会有其他情况)
    auto* pkt = newBluetoothRawPacket(pktSize);
    if (pkt == nullptr) {
        freeSubPacket(hapticData, subPacketType::haptic);
        freeSubPacket(statusData, subPacketType::status);
        freeSubPacket(audioData, subPacketType::audio);
        return nullptr;
    }

    packed(hapticData, statusData, audioData, pkt);
    freeSubPacket(hapticData, subPacketType::haptic);
    freeSubPacket(statusData, subPacketType::status);
    freeSubPacket(audioData, subPacketType::audio);

    *size = pkt->size;
    return pkt->data;
}
