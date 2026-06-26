//
// Created by awalol on 2026/3/4.
//

#include "bt.h"

#include <assert.h>
#include <bsp/board_api.h>
#include <btstack_event.h>
#include <classic/sdp_server.h>
#include <hardware/timer.h>
#include <hci_cmd.h>
#include <l2cap.h>
#include <pico/cyw43_arch.h>
#include <pico/platform/compiler.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "audio.h"
#include "bluetooth_packet.h"
#include "config.h"
#include "crc32.h"
#include "log.h"
#include "state.h"

#define MTU_CONTROL 672
#define MTU_INTERRUPT 672

// DualSense 通过 BR/EDR HID 的蓝牙控制面与数据面。
// - HCI 回调负责搜索、配对/鉴权以及 ACL 生命周期。
// - L2CAP 回调负责 HID control/interrupt 信道与数据收发。

static void l2capPacketHandler(uint8_t packet_type, uint16_t channel, uint8_t* packet, uint16_t size);
static void hciPacketHandler(uint8_t packet_type, uint16_t channel, uint8_t* packet, uint16_t size);

static btstack_packet_callback_registration_t hciEventCallbackRegistration, l2capEventCallbackRegistration;

typedef struct {
    // 最近一次在搜索结果或入站 ACL 请求中看到的对端地址。
    bd_addr_t currentDeviceAddr;
    bool deviceFound;
    bool newPair;  // 只有新匹配的设备才用创建channel，自动重连走的是service
    hci_con_handle_t aclHandle;
    uint16_t hidControlCid;
    uint16_t hidInterruptCid;
    bool checkDse;
    bool requestHasBeenSent;
    bool inquiring;
    absolute_time_t inactiveTime;  // 手柄长时间静默
} BtRuntime;

static BtRuntime bt = {
    .aclHandle = HCI_CON_HANDLE_INVALID,
};

constexpr uint8_t featureSlotCount = 16;
constexpr uint8_t featureDataMax = 64;
struct FeatureSlot {
    bool valid;
    uint8_t len;                   // 实际存储字节数（包含 data[0] = reportId）
    uint8_t data[featureDataMax];  // data[0]=reportId, data[1..len-1]=payload
};
static struct FeatureSlot featureData[featureSlotCount] = {};

// 找到对应 reportId 的槽位，未找到返回 nullptr
static inline struct FeatureSlot* __not_in_flash_func(findFeatureSlot)(const uint8_t reportId) {
    for (int i = 0; i < featureSlotCount; ++i) {
        if (featureData[i].valid && featureData[i].data[0] == reportId) {
            return &featureData[i];
        }
    }
    return nullptr;
}

// 找到或分配一个槽位（同 reportId 复用）
static inline struct FeatureSlot* __not_in_flash_func(allocFeatureSlot)(const uint8_t reportId) {
    // 同 reportId 复用
    for (int i = 0; i < featureSlotCount; ++i) {
        if (featureData[i].valid && featureData[i].data[0] == reportId) {
            return &featureData[i];
        }
    }

    // 空槽
    for (int i = 0; i < featureSlotCount; ++i) {
        if (!featureData[i].valid) {
            return &featureData[i];
        }
    }
    return nullptr;  // 槽位已满
}

static inline void clearFeatureCache() {
    for (int i = 0; i < featureSlotCount; ++i) {
        featureData[i].valid = false;
        featureData[i].len = 0;
    }
}

static inline bool isGamepadCod(const uint32_t cod) {
    // CoD 0x002508 = Gamepad (Major: Peripheral, Minor: Gamepad)
    return (cod & 0x000F00) == 0x000500;
}

static inline bool btDisconnect() {
    if (bt.aclHandle == HCI_CON_HANDLE_INVALID) {
        return false;
    }

    // 0x13 = 远端用户终止连接
    hci_send_cmd(&hci_disconnect, bt.aclHandle, 0x13);
    return true;
}

static inline void btL2capInit() {
    l2capEventCallbackRegistration.callback = &l2capPacketHandler;
    l2cap_add_event_handler(&l2capEventCallbackRegistration);
    // 修复重连后自动断开的关键点
    sdp_init();
    l2cap_register_service(l2capPacketHandler, PSM_HID_CONTROL, MTU_CONTROL, LEVEL_2);
    l2cap_register_service(l2capPacketHandler, PSM_HID_INTERRUPT, MTU_INTERRUPT, LEVEL_2);

    l2cap_init();
}

int btInit() {
    btL2capInit();

    // SSP（安全简单配对）
    gap_ssp_set_enable(1);
    gap_secure_connections_enable(true);
    gap_ssp_set_io_capability(SSP_IO_CAPABILITY_DISPLAY_YES_NO);
    gap_ssp_set_authentication_requirement(SSP_IO_AUTHREQ_MITM_PROTECTION_NOT_REQUIRED_GENERAL_BONDING);

    gap_set_page_scan_activity(0x0012, 0x0012);  // 11.25ms
    gap_set_page_scan_type(PAGE_SCAN_MODE_INTERLACED);
    gap_connectable_control(1);
    gap_discoverable_control(1);

    hciEventCallbackRegistration.callback = &hciPacketHandler;
    hci_add_event_handler(&hciEventCallbackRegistration);

    hci_power_control(HCI_POWER_ON);
    return 0;
}

void __not_in_flash_func(btInquiringLed)() {
    if (bt.hidInterruptCid != 0) {
        return;
    }
    static bool ledStatus = false;
    if (!bt.inquiring) {
        if (ledStatus) {
            cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, false);
        }
        return;
    }

    static uint32_t lastTime = 0;
    if (time_us_32() - lastTime > 200 * 1000) {
        lastTime = time_us_32();
        ledStatus = !ledStatus;
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, ledStatus);
    }
}

static inline void hciHandleBtstackState(uint8_t* packet) {
    const uint8_t state = btstack_event_state_get_state(packet);
    LOGI("[BT] State: %u", state);
    if (state == HCI_STATE_WORKING) {
        LOGI("[BT] Stack ready, start inquiry");
        gap_inquiry_start(30);
        bt.inquiring = true;
    }
}

static inline void hciHandleInquiryResult(const uint8_t eventType, uint8_t* packet) {
    bd_addr_t addr;
    uint32_t cod = 0;

    if (eventType == HCI_EVENT_INQUIRY_RESULT) {
        cod = hci_event_inquiry_result_get_class_of_device(packet);
        hci_event_inquiry_result_get_bd_addr(packet, addr);
    } else if (eventType == HCI_EVENT_INQUIRY_RESULT_WITH_RSSI) {
        cod = hci_event_inquiry_result_with_rssi_get_class_of_device(packet);
        hci_event_inquiry_result_with_rssi_get_bd_addr(packet, addr);
    } else {
        cod = hci_event_extended_inquiry_response_get_class_of_device(packet);
        hci_event_extended_inquiry_response_get_bd_addr(packet, addr);
    }

    if (isGamepadCod(cod)) {
        LOGI("[HCI] Gamepad found: %s (CoD: 0x%06x)", bd_addr_to_str(addr), (unsigned int)cod);
        bd_addr_copy(bt.currentDeviceAddr, addr);
        bt.deviceFound = true;
        gap_inquiry_stop();
    }
}

static inline void hciHandleInquiryComplete(const uint8_t eventType) {
    LOGI("[HCI] Inquiry complete.");
    bt.inquiring = false;
    if (bt.deviceFound) {
        LOGI("[HCI] Connecting to %s...", bd_addr_to_str(bt.currentDeviceAddr));
        bt.newPair = true;
        hci_send_cmd(&hci_create_connection, bt.currentDeviceAddr, hci_usable_acl_packet_types(), 0, 0, 0, 1);
        return;
    }
    if (eventType == HCI_EVENT_INQUIRY_COMPLETE) {
        gap_connectable_control(1);
        gap_discoverable_control(1);
    }
}

static inline void hciHandleCommandStatus(uint8_t* packet) {
    const uint8_t status = hci_event_command_status_get_status(packet);
    const uint16_t opcode = hci_event_command_status_get_command_opcode(packet);
    LOGI("[HCI] CmdStatus (0x%04X) status=0x%02X", opcode, status);
    if (opcode == HCI_OPCODE_HCI_CREATE_CONNECTION && status != ERROR_CODE_SUCCESS) {
        bt.deviceFound = false;
        bt.newPair = false;
        LOGW("[HCI] Create connection rejected");
    }
    if (opcode == HCI_OPCODE_HCI_INQUIRY_CANCEL) {
        bt.inquiring = false;
    }
}

static inline void hciHandleCommandComplete(uint8_t* packet) {
    [[maybe_unused]] const uint8_t status = hci_event_command_complete_get_return_parameters(packet)[0];
    [[maybe_unused]] const uint16_t opcode = hci_event_command_complete_get_command_opcode(packet);
    LOGI("[HCI] CmdComplete (0x%04X) status=0x%02X", opcode, status);
}

static inline void hciHandleConnectionComplete(uint8_t* packet) {
    const uint8_t status = hci_event_connection_complete_get_status(packet);
    if (status == 0) {
        const hci_con_handle_t handle = hci_event_connection_complete_get_connection_handle(packet);
        bt.aclHandle = handle;
        hci_event_connection_complete_get_bd_addr(packet, bt.currentDeviceAddr);
        LOGI("[HCI] ACL connected handle=0x%04X", handle);
        LOGI("[HCI] Request authentication on handle=0x%04X", handle);
        hci_send_cmd(&hci_authentication_requested, handle);
    } else {
        bt.deviceFound = false;
        bt.newPair = false;
        LOGW("[HCI] ACL connect failed status=0x%02X", status);
    }
}

static inline void hciHandleLinkKeyRequest(uint8_t* packet) {
    bd_addr_t addr;
    hci_event_link_key_request_get_bd_addr(packet, addr);
    link_key_t linkKey = {};
    link_key_type_t linkKeyType = {};
    const bool link = gap_get_link_key_for_bd_addr(addr, linkKey, &linkKeyType);
    if (link) {
        LOGD("[HCI] Link key: ");
        printHex(linkKey, sizeof(linkKey));

        LOGI("[HCI] Link key request from %s, reply stored key type=%u", bd_addr_to_str(addr), (unsigned int)linkKeyType);
        hci_send_cmd(&hci_link_key_request_reply, addr, linkKey);
    } else {
        LOGI("[HCI] Link key request from %s, no key, force re-pair", bd_addr_to_str(addr));
        hci_send_cmd(&hci_link_key_request_negative_reply, addr);
    }
}

static inline void hciHandleUserConfirmationRequest(uint8_t* packet) {
    bd_addr_t addr;
    hci_event_user_confirmation_request_get_bd_addr(packet, addr);
    LOGI("[HCI] User confirmation request from %s, accept", bd_addr_to_str(addr));
    hci_send_cmd(&hci_user_confirmation_request_reply, addr);
}

static inline void hciHandlePinCodeRequest(uint8_t* packet) {
    bd_addr_t addr;
    hci_event_pin_code_request_get_bd_addr(packet, addr);
    LOGI("[HCI] Legacy pin request from %s, reply 0000", bd_addr_to_str(addr));
    gap_pin_code_response(addr, "0000");
}

static inline void hciHandleAuthenticationComplete(uint8_t* packet) {
    const uint8_t status = hci_event_authentication_complete_get_status(packet);
    const hci_con_handle_t handle = hci_event_authentication_complete_get_connection_handle(packet);
    LOGI("[HCI] Authentication complete handle=0x%04X status=0x%02X", handle, status);
    if (status != ERROR_CODE_SUCCESS) {
        LOGW("[HCI] Authentication failed, drop stored key for %s", bd_addr_to_str(bt.currentDeviceAddr));
        gap_drop_link_key_for_bd_addr(bt.currentDeviceAddr);
    } else {
        hci_send_cmd(&hci_set_connection_encryption, handle, 1);
    }
}

static inline void hciHandleEncryptionChange(uint8_t* packet) {
    const uint8_t status = hci_event_encryption_change_get_status(packet);
    [[maybe_unused]] const hci_con_handle_t handle = hci_event_encryption_change_get_connection_handle(packet);
    const uint8_t enabled = hci_event_encryption_change_get_encryption_enabled(packet);
    LOGI("[HCI] Encryption change handle=0x%04X status=0x%02X enabled=%u", handle, status, enabled);
    if (status == ERROR_CODE_SUCCESS && (enabled != 0U)) {
        LOGI("[L2CAP] Open HID channels");
        // 仅在首次配对流程中主动打开 HID 信道；
        // 对于基于 service 的自动重连，走被动接入路径。
        if (bt.newPair) {
            if (bt.hidControlCid == 0) {
                l2cap_create_channel(l2capPacketHandler, bt.currentDeviceAddr, PSM_HID_CONTROL, MTU_CONTROL, &bt.hidControlCid);
            } else if (bt.hidInterruptCid == 0) {
                l2cap_create_channel(l2capPacketHandler, bt.currentDeviceAddr, PSM_HID_INTERRUPT, MTU_INTERRUPT, &bt.hidInterruptCid);
            }
        }
    }
}

static inline void hciHandleConnectionRequest(uint8_t* packet) {
    bd_addr_t addr;
    hci_event_connection_request_get_bd_addr(packet, addr);
    const uint32_t cod = hci_event_connection_request_get_class_of_device(packet);
    LOGI("[HCI] Incoming ACL request from %s cod=0x%06x", bd_addr_to_str(addr), (unsigned int)cod);
    if (isGamepadCod(cod)) {
        bd_addr_copy(bt.currentDeviceAddr, addr);
        gap_inquiry_stop();
        hci_send_cmd(&hci_accept_connection_request, addr, 0x01);
    }
}

static inline void hciHandleDisconnectionComplete(uint8_t* packet) {
    tud_disconnect();
    gap_connectable_control(1);
    gap_discoverable_control(1);
    [[maybe_unused]] const uint8_t reason = hci_event_disconnection_complete_get_reason(packet);
    bt.deviceFound = false;
    bt.newPair = false;
    bt.aclHandle = HCI_CON_HANDLE_INVALID;
    bt.hidControlCid = 0;
    bt.hidInterruptCid = 0;
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, false);
    LOGI("[HCI] Disconnected reason=0x%02X", reason);
}

static void __not_in_flash_func(hciPacketHandler)(uint8_t packet_type, uint16_t channel, uint8_t* packet, uint16_t size) {
    (void)packet_type;
    (void)channel;
    (void)size;

    const uint8_t eventType = hci_event_packet_get_type(packet);
    switch (eventType) {
        case BTSTACK_EVENT_STATE:
            hciHandleBtstackState(packet);
            break;
        case HCI_EVENT_INQUIRY_RESULT:
        case HCI_EVENT_INQUIRY_RESULT_WITH_RSSI:
        case HCI_EVENT_EXTENDED_INQUIRY_RESPONSE:
            hciHandleInquiryResult(eventType, packet);
            break;
        case GAP_EVENT_INQUIRY_COMPLETE:
        case HCI_EVENT_INQUIRY_COMPLETE:
            hciHandleInquiryComplete(eventType);
            break;
        case HCI_EVENT_COMMAND_STATUS:
            hciHandleCommandStatus(packet);
            break;
        case HCI_EVENT_COMMAND_COMPLETE:
            hciHandleCommandComplete(packet);
            break;
        case HCI_EVENT_CONNECTION_COMPLETE:
            hciHandleConnectionComplete(packet);
            break;
        case HCI_EVENT_LINK_KEY_REQUEST:
            hciHandleLinkKeyRequest(packet);
            break;
        case HCI_EVENT_USER_CONFIRMATION_REQUEST:
            hciHandleUserConfirmationRequest(packet);
            break;
        case HCI_EVENT_PIN_CODE_REQUEST:
            hciHandlePinCodeRequest(packet);
            break;
        case HCI_EVENT_AUTHENTICATION_COMPLETE:
            hciHandleAuthenticationComplete(packet);
            break;
        case HCI_EVENT_ENCRYPTION_CHANGE:
            hciHandleEncryptionChange(packet);
            break;
        case HCI_EVENT_CONNECTION_REQUEST:
            hciHandleConnectionRequest(packet);
            break;
        case HCI_EVENT_DISCONNECTION_COMPLETE:
            hciHandleDisconnectionComplete(packet);
            break;
        default:
            break;
    }
}

static inline void __not_in_flash_func(l2capHandleInterruptDataPacket)(uint8_t* packet, const uint16_t size) {
    if (size <= 4 || packet[1] != 0x31) {
        return;
    }

    // Mic audio: controller signals mic payload via bit1 of packet[2];
    // the opus-encoded mic frame starts at packet+4.
    if (0 != (packet[2] & (1 << 1))) {
        micAddOpusQueue(packet + 4, size - 4);
        return;
    }

    // state packet
    if (size - 3 < ds5StatePacketSize) {
        return;
    }
    union Ds5StateUnion* ds5StateUnion = (union Ds5StateUnion*)(packet + 3);
    setStatePacket(ds5StateUnion);

    // 静默检测
    if (config.micActive || ds5StateUnion->packet.LeftStickX < 120 || ds5StateUnion->packet.LeftStickX > 140 || ds5StateUnion->packet.LeftStickY < 120 || ds5StateUnion->packet.LeftStickY > 140 ||
        ds5StateUnion->packet.RightStickX < 120 || ds5StateUnion->packet.RightStickX > 140 || ds5StateUnion->packet.RightStickY < 120 || ds5StateUnion->packet.RightStickY > 140 ||
        ds5StateUnion->packet.TriggerLeft > 0 || ds5StateUnion->packet.TriggerRight > 0 || ds5StateUnion->data[7] != DirectionNone || ds5StateUnion->data[8] != 0x00 ||
        ds5StateUnion->data[9] != 0x00) {
        bt.inactiveTime = get_absolute_time();
    } else if (absolute_time_diff_us(bt.inactiveTime, get_absolute_time()) > (int64_t)(config.inactiveTime) * 60 * 1000 * 1000) {
        // config.inactiveTime 的单位是分钟；这里按微秒比较。
        LOGI("disconnect when inactive");
        btDisconnect();
    }
}

static inline void __not_in_flash_func(l2capHandleControlDataPacket)(uint8_t* packet, const uint16_t size) {
    if (size < 1) {
        LOGW("[L2CAP] HID Control data empty");
        return;
    }
    if (bt.checkDse) {
        // checkDse 由 initFeature()（请求 0x70 report）置位，
        // 在首个匹配的 control 响应中消费。
        // packet[0] == 0x02 只需 1 字节
        if (packet[0] == 0x02) {
            LOGI("Connected DS5 Controller");
            bt.checkDse = false;
            config.isDse = false;
            tud_connect();
        } else if (size >= 2 && packet[0] == 0xA3 && packet[1] == 0x70) {
            // 需要 2 字节才能读 packet[1]
            LOGI("Connected DSE Controller");
            bt.checkDse = false;
            config.isDse = true;
            tud_connect();
        }
    }
    // 存储 Feature Report：packet[1] 是 reportId，需要 size >= 2
    if (size >= 2 && packet[0] == 0xA3) {
        const uint8_t reportId = packet[1];
        // 存 packet[1..size-1]（含 reportId），截断到 featureDataMax
        const uint8_t storeLen = (uint8_t)MIN((size - 1), (uint16_t)featureDataMax);
        struct FeatureSlot* slot = allocFeatureSlot(reportId);
        if (slot != nullptr) {
            memcpy(slot->data, packet + 1, storeLen);
            slot->len = storeLen;
            slot->valid = true;
        } else {
            LOGW("[L2CAP] Feature slot full, drop Report 0x%02X", reportId);
        }
        LOGD("[L2CAP] Stored Feature Report 0x%02X, len=%u", reportId, storeLen - 1);
    }

    LOGD("[L2CAP] HID Control data len=%u", size);
#if ENABLE_DEBUG
    printf_hexdump(packet, size);
#endif
}

static inline void __not_in_flash_func(l2capHandleDataPacket)(const uint16_t channel, uint8_t* packet, const uint16_t size) {
    // 数据路径分发：interrupt 信道承载输入/麦克风，
    // control 信道承载 feature report 与手柄能力响应。
    if (channel == bt.hidInterruptCid) {
        l2capHandleInterruptDataPacket(packet, size);
        return;
    }

    if (channel == bt.hidControlCid) {
        l2capHandleControlDataPacket(packet, size);
        return;
    }

    LOGE("[L2CAP] Data on unknown channel 0x%04X (Interrupt: 0x%04X, Control: 0x%04X)", channel, bt.hidInterruptCid, bt.hidControlCid);
}

static inline void __not_in_flash_func(l2capHandleChannelOpened)(uint8_t* packet) {
    const uint8_t status = l2cap_event_channel_opened_get_status(packet);
    const uint16_t localCid = l2cap_event_channel_opened_get_local_cid(packet);
    if (status == 0) {
        const uint16_t psm = l2cap_event_channel_opened_get_psm(packet);
        if (psm == PSM_HID_CONTROL) {
            LOGI("[L2CAP] HID Control opened cid=0x%04X", localCid);
            bt.hidControlCid = localCid;

            [[maybe_unused]] const uint16_t mtu = l2cap_get_remote_mtu_for_local_cid(bt.hidControlCid);
            LOGI("[L2CAP] Remote Control MTU: %d", mtu);
        } else if (psm == PSM_HID_INTERRUPT) {
            LOGI("[L2CAP] HID Interrupt opened cid=0x%04X", localCid);
            bt.hidInterruptCid = localCid;
            cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, true);
            bt.inactiveTime = get_absolute_time();

            LOGI("Init DualSense");
            initFeature();
            // 初始化手柄状态
            setControlPacket(ds5ControlInitPacket.data, ds5ControlPacketSize);

            [[maybe_unused]] const uint16_t mtu = l2cap_get_remote_mtu_for_local_cid(bt.hidInterruptCid);
            LOGI("[L2CAP] Remote Interrupt MTU: %d", mtu);

            gap_connectable_control(0U);
            gap_discoverable_control(0U);
        } else {
            LOGE("[L2CAP] Unknown Channel psm: 0x%02X", psm);
        }

        return;
    }

    [[maybe_unused]] const uint16_t psm = l2cap_event_channel_opened_get_psm(packet);
    bt.hidControlCid = 0;
    bt.hidInterruptCid = 0;
    bt.deviceFound = false;
    LOGI("[L2CAP] Open failed psm=0x%04X status=0x%02X", psm, status);
    btDisconnect();
}

static inline void __not_in_flash_func(l2capHandleIncomingConnection)(uint8_t* packet) {
    const uint16_t localCid = l2cap_event_incoming_connection_get_local_cid(packet);
    [[maybe_unused]] const uint16_t psm = l2cap_event_incoming_connection_get_psm(packet);
    LOGI("[L2CAP] Incoming connection psm=0x%04X cid=0x%04X", psm, localCid);
    l2cap_accept_connection(localCid);
}

static inline void __not_in_flash_func(l2capHandleChannelClosed)(uint8_t* packet) {
    const uint16_t localCid = l2cap_event_channel_closed_get_local_cid(packet);
    if (localCid == bt.hidControlCid) {
        bt.hidControlCid = 0;
        LOGI("[L2CAP] HID Control closed cid=0x%04X", localCid);
    } else if (localCid == bt.hidInterruptCid) {
        bt.hidInterruptCid = 0;
        LOGI("[L2CAP] HID Interrupt closed cid=0x%04X", localCid);
    } else {
        LOGI("[L2CAP] Channel closed cid=0x%04X", localCid);
    }
    if (bt.hidControlCid == 0 && bt.hidInterruptCid == 0) {
        btDisconnect();
    }
}

static inline void __not_in_flash_func(l2capHandleCanSendNow)(void) {
    bt.requestHasBeenSent = false;
    size_t len = 0;
    uint8_t* bluetoothRawPacket = getBluetoothRawPacket(&len);
    if (bluetoothRawPacket != nullptr) {
        const uint8_t status = l2cap_send(bt.hidInterruptCid, bluetoothRawPacket, len);
        if (status != 0) {
            LOGE("[L2CAP] L2CAP Send Error, Status: 0x%02X", status);
        }
        freeBluetoothRawPacket(bluetoothRawPacket);
    }

    btRequestSend();
}

static inline void __not_in_flash_func(l2capHandleEventPacket)(uint8_t* packet) {
    // 事件路径分发：处理 L2CAP 信道生命周期与发送调度。
    const uint8_t eventType = hci_event_packet_get_type(packet);
    switch (eventType) {
        case L2CAP_EVENT_CHANNEL_OPENED:
            l2capHandleChannelOpened(packet);
            break;
        case L2CAP_EVENT_INCOMING_CONNECTION:
            l2capHandleIncomingConnection(packet);
            break;
        case L2CAP_EVENT_CHANNEL_CLOSED:
            l2capHandleChannelClosed(packet);
            break;
        case L2CAP_EVENT_CAN_SEND_NOW:
            l2capHandleCanSendNow();
            break;
        default:
            break;
    }
}

static void __not_in_flash_func(l2capPacketHandler)(const uint8_t packet_type, const uint16_t channel, uint8_t* packet, const uint16_t size) {
    if (packet_type == L2CAP_DATA_PACKET) {
        l2capHandleDataPacket(channel, packet, size);
        return;
    }

    l2capHandleEventPacket(packet);
}

void __not_in_flash_func(btRequestSend)() {
    if (bt.hidInterruptCid != 0 && !bt.requestHasBeenSent && hasBluetoothRawPacketCanSend()) {
        bt.requestHasBeenSent = true;
        l2cap_request_can_send_now_event(bt.hidInterruptCid);
    }
}

uint16_t __not_in_flash_func(getFeatureData)(const uint8_t reportId, uint8_t* outBuf, const uint16_t maxLen) {
    // 若为0x81则会请求新内容，其他若有旧数据则不进行请求
    uint16_t copied = 0;
    const struct FeatureSlot* slot = findFeatureSlot(reportId);
    const bool hasData = (slot != nullptr);
    if (hasData && outBuf != nullptr && slot->len > 1) {
        // slot->data[0] = reportId，跳过，直接把 payload 写入调用方 buffer
        copied = (uint16_t)MIN((uint16_t)(slot->len - 1), maxLen);
        memcpy(outBuf, slot->data + 1, copied);
    }

    if (!hasData ||
        // Get Test Command Result
        reportId == 0x81 ||
        // DSE: Set Profile Save?
        reportId == 0x63 || reportId == 0x65 || reportId == 0x64) {
        if (bt.hidControlCid != 0) {
            const uint8_t getFeature[] = {0x43, reportId};
            l2cap_send(bt.hidControlCid, getFeature, sizeof(getFeature));
            LOGD("[L2CAP] Requesting Get Feature Report 0x%02X", reportId);
        }
    }
    return copied;
}

void __not_in_flash_func(setFeatureData)(const uint8_t reportId, const uint8_t* data, const uint16_t len) {
    if (bt.hidControlCid != 0) {
        static uint8_t getFeature[MTU_CONTROL];
        constexpr size_t featureCrcSize = 4;
        if (len < featureCrcSize) {
            LOGE("[L2CAP] set_feature_data len too small:%u", len);
            return;
        }
        if (data == nullptr) {
            LOGE("[L2CAP] set_feature_data data is nullptr");
            return;
        }
        if ((size_t)len + 2 > sizeof(getFeature)) {
            LOGE("[L2CAP] set_feature_data len too large:%u", len);
            return;
        }
        getFeature[0] = 0x53;
        getFeature[1] = reportId;
        memcpy(getFeature + 2, data, len);
        fillFeatureReportChecksum(getFeature + 1, len + 1);
        l2cap_send(bt.hidControlCid, getFeature, len + 2);

        LOGD("[L2CAP] Requesting Set Feature Report 0x%02X", reportId);
#if ENABLE_DEBUG
        printf_hexdump(getFeature, len + 2);
#endif
    }
}

void btPowerOffController() {
    uint8_t bluetoothControl[47] = {};
    bluetoothControl[0] = 0x02;  // DualSense Bluetooth control: 1=on, 2=off.
    setFeatureData(0x08, bluetoothControl, sizeof(bluetoothControl));
}

void initFeature() {
    clearFeatureCache();
    getFeatureData(0x09, nullptr, 0);
    getFeatureData(0x20, nullptr, 0);
    getFeatureData(0x22, nullptr, 0);
    getFeatureData(0x05, nullptr, 0);
    getFeatureData(0x81, nullptr, 0);
    // 当 0x70 report 响应到达时，checkDse 会在 l2capHandleControlDataPacket()
    // 中被消费：DS5 路径清为 false；DSE 路径会设置 config.isDse。
    bt.checkDse = true;
    getFeatureData(0x70, nullptr, 0);
}
