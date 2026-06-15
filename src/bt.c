//
// Created by awalol on 2026/3/4.
//

#include "bt.h"

#include <assert.h>
#include <bsp/board_api.h>
#include <btstack_event.h>
#include <classic/sdp_server.h>
#include <l2cap.h>
#include <pico/cyw43_arch.h>
#include <pico/platform/compiler.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "bluetoothPacket.h"
#include "config.h"
#include "crc32.h"
#include "log.h"
#include "state.h"

#define MTU_CONTROL 672
#define MTU_INTERRUPT 672

static void hciPacketHandler(uint8_t packet_type, uint16_t channel, uint8_t* packet, uint16_t size);
static void l2capPacketHandler(uint8_t packet_type, uint16_t channel, uint8_t* packet, uint16_t size);

static btstack_packet_callback_registration_t hciEventCallbackRegistration, l2capEventCallbackRegistration;
static bd_addr_t currentDeviceAddr;
static bool deviceFound = false;
static bool newPair = false;  // 只有新匹配的设备才用创建channel，自动重连走的是service
static hci_con_handle_t aclHandle = HCI_CON_HANDLE_INVALID;
static uint16_t hidControlCid;
static uint16_t hidInterruptCid;
static bool checkDse = false;
static bool theRequestHasBeenSent = false;

// 静态 Feature Report 缓存，替代 std::unordered_map + std::vector，消除堆分配
// 每条最大存 64 字节（含首字节 reportId），16 个槽位覆盖全部 DS5 feature report
constexpr uint8_t featureSlotCount = 16;
constexpr uint8_t featureDataMax = 64;
struct FeatureSlot {
    bool valid;
    uint8_t len;                   // 实际存储字节数（包含 data[0] = reportId）
    uint8_t data[featureDataMax];  // data[0]=reportId, data[1..len-1]=payload
};
static struct FeatureSlot featureData[featureSlotCount] = {};

// 找到对应 reportId 的槽位，未找到返回 nullptr
static struct FeatureSlot* findFeatureSlot(const uint8_t reportId) {
    for (int i = 0; i < featureSlotCount; ++i) {
        if (featureData[i].valid && featureData[i].data[0] == reportId) {
            return &featureData[i];
        }
    }
    return nullptr;
}

// 找到或分配一个槽位（同 reportId 复用）
static struct FeatureSlot* allocFeatureSlot(const uint8_t reportId) {
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

absolute_time_t inactiveTime = 0;  // 手柄长时间静默

static inline bool btDisconnect() {
    if (aclHandle == HCI_CON_HANDLE_INVALID) {
        return false;
    }

    // 0x13 = remote user terminated connection
    hci_send_cmd(&hci_disconnect, aclHandle, 0x13);
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
    for (int i = 0; i < featureSlotCount; i++) {
        featureData[i].valid = false;
        featureData[i].len = 0;
    }

    btL2capInit();

    // SSP (Secure Simple Pairing)
    gap_ssp_set_enable(1);
    gap_secure_connections_enable(true);
    gap_ssp_set_io_capability(SSP_IO_CAPABILITY_DISPLAY_YES_NO);
    gap_ssp_set_authentication_requirement(SSP_IO_AUTHREQ_MITM_PROTECTION_NOT_REQUIRED_GENERAL_BONDING);

    gap_connectable_control(1);
    gap_discoverable_control(1);

    hciEventCallbackRegistration.callback = &hciPacketHandler;
    hci_add_event_handler(&hciEventCallbackRegistration);

    hci_power_control(HCI_POWER_ON);
    return 0;
}

static void __not_in_flash_func(hciPacketHandler)(uint8_t packet_type, uint16_t channel, uint8_t* packet, uint16_t size) {
    const uint8_t eventType = hci_event_packet_get_type(packet);
    switch (eventType) {
        case BTSTACK_EVENT_STATE: {
            const uint8_t state = btstack_event_state_get_state(packet);
            LOGI("[BT] State: %u", state);
            if (state == HCI_STATE_WORKING) {
                LOGI("[BT] Stack ready, start inquiry");
                gap_inquiry_start(30);
            }
            break;
        }
        case HCI_EVENT_INQUIRY_RESULT:
        case HCI_EVENT_INQUIRY_RESULT_WITH_RSSI:
        case HCI_EVENT_EXTENDED_INQUIRY_RESPONSE: {
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

            // CoD 0x002508 = Gamepad (Major: Peripheral, Minor: Gamepad)
            if ((cod & 0x000F00) == 0x000500) {
                LOGI("[HCI] Gamepad found: %s (CoD: 0x%06x)", bd_addr_to_str(addr), (unsigned int)cod);
                bd_addr_copy(currentDeviceAddr, addr);
                deviceFound = true;
                gap_inquiry_stop();
            }
            break;
        }

        case GAP_EVENT_INQUIRY_COMPLETE:
        case HCI_EVENT_INQUIRY_COMPLETE: {
            LOGI("[HCI] Inquiry complete.");
            if (deviceFound) {
                LOGI("[HCI] Connecting to %s...", bd_addr_to_str(currentDeviceAddr));
                newPair = true;
                hci_send_cmd(&hci_create_connection, currentDeviceAddr, hci_usable_acl_packet_types(), 0, 0, 0, 1);
                break;
            }
            if (eventType == HCI_EVENT_INQUIRY_COMPLETE) {
                LOGI("[HCI] Restart inquiry");
                gap_inquiry_start(30);
                gap_connectable_control(1);
                gap_discoverable_control(1);
            }
            break;
        }
        case HCI_EVENT_COMMAND_STATUS: {
            const uint8_t status = hci_event_command_status_get_status(packet);
            const uint16_t opcode = hci_event_command_status_get_command_opcode(packet);
            LOGI("[HCI] CmdStatus (0x%04X) status=0x%02X", opcode, status);
            if (opcode == HCI_OPCODE_HCI_CREATE_CONNECTION && status != ERROR_CODE_SUCCESS) {
                deviceFound = false;
                newPair = false;
                LOGW("[HCI] Create connection rejected, restart inquiry");
                // gap_inquiry_start(30);
            }
            break;
        }

        case HCI_EVENT_COMMAND_COMPLETE: {
            [[maybe_unused]] const uint8_t status = hci_event_command_complete_get_return_parameters(packet)[0];
            [[maybe_unused]] const uint16_t opcode = hci_event_command_complete_get_command_opcode(packet);
            LOGI("[HCI] CmdComplete (0x%04X) status=0x%02X", opcode, status);
            break;
        }

        case HCI_EVENT_CONNECTION_COMPLETE: {
            const uint8_t status = hci_event_connection_complete_get_status(packet);
            if (status == 0) {
                const hci_con_handle_t handle = hci_event_connection_complete_get_connection_handle(packet);
                aclHandle = handle;
                hci_event_connection_complete_get_bd_addr(packet, currentDeviceAddr);
                LOGI("[HCI] ACL connected handle=0x%04X", handle);
                LOGI("[HCI] Request authentication on handle=0x%04X", handle);
                hci_send_cmd(&hci_authentication_requested, handle);
            } else {
                deviceFound = false;
                newPair = false;
                LOGW("[HCI] ACL connect failed status=0x%02X, restart inquiry", status);
                // gap_inquiry_start(30);
            }
            break;
        }

        case HCI_EVENT_LINK_KEY_REQUEST: {
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
            break;
        }

        case HCI_EVENT_USER_CONFIRMATION_REQUEST: {
            bd_addr_t addr;
            hci_event_user_confirmation_request_get_bd_addr(packet, addr);
            LOGI("[HCI] User confirmation request from %s, accept", bd_addr_to_str(addr));
            hci_send_cmd(&hci_user_confirmation_request_reply, addr);
            break;
        }

        case HCI_EVENT_PIN_CODE_REQUEST: {
            bd_addr_t addr;
            hci_event_pin_code_request_get_bd_addr(packet, addr);
            LOGI("[HCI] Legacy pin request from %s, reply 0000", bd_addr_to_str(addr));
            gap_pin_code_response(addr, "0000");
            break;
        }

        case HCI_EVENT_AUTHENTICATION_COMPLETE: {
            const uint8_t status = hci_event_authentication_complete_get_status(packet);
            const hci_con_handle_t handle = hci_event_authentication_complete_get_connection_handle(packet);
            LOGI("[HCI] Authentication complete handle=0x%04X status=0x%02X", handle, status);
            if (status != ERROR_CODE_SUCCESS) {
                LOGW("[HCI] Authentication failed, drop stored key for %s", bd_addr_to_str(currentDeviceAddr));
                gap_drop_link_key_for_bd_addr(currentDeviceAddr);
                // gap_inquiry_start(30);
            } else {
                hci_send_cmd(&hci_set_connection_encryption, handle, 1);
            }
            break;
        }

        case HCI_EVENT_ENCRYPTION_CHANGE: {
            const uint8_t status = hci_event_encryption_change_get_status(packet);
            [[maybe_unused]] const hci_con_handle_t handle = hci_event_encryption_change_get_connection_handle(packet);
            const uint8_t enabled = hci_event_encryption_change_get_encryption_enabled(packet);
            LOGI("[HCI] Encryption change handle=0x%04X status=0x%02X enabled=%u", handle, status, enabled);
            if (status == ERROR_CODE_SUCCESS && (enabled != 0U)) {
                LOGI("[L2CAP] Open HID channels");
                if (newPair) {
                    if (hidControlCid == 0) {
                        l2cap_create_channel(l2capPacketHandler, currentDeviceAddr, PSM_HID_CONTROL, MTU_CONTROL, &hidControlCid);
                    } else if (hidInterruptCid == 0) {
                        l2cap_create_channel(l2capPacketHandler, currentDeviceAddr, PSM_HID_INTERRUPT, MTU_INTERRUPT, &hidInterruptCid);
                    }
                }
            }
            break;
        }

        case HCI_EVENT_CONNECTION_REQUEST: {
            bd_addr_t addr;
            hci_event_connection_request_get_bd_addr(packet, addr);
            const uint32_t cod = hci_event_connection_request_get_class_of_device(packet);
            LOGI("[HCI] Incoming ACL request from %s cod=0x%06x", bd_addr_to_str(addr), (unsigned int)cod);
            if ((cod & 0x000F00) == 0x000500) {
                bd_addr_copy(currentDeviceAddr, addr);
                gap_inquiry_stop();
                hci_send_cmd(&hci_accept_connection_request, addr, 0x01);
            }
            break;
        }

        case HCI_EVENT_DISCONNECTION_COMPLETE: {
            tud_disconnect();
            gap_connectable_control(1);
            gap_discoverable_control(1);
            [[maybe_unused]] const uint8_t reason = hci_event_disconnection_complete_get_reason(packet);
            deviceFound = false;
            newPair = false;
            aclHandle = HCI_CON_HANDLE_INVALID;
            hidControlCid = 0;
            hidInterruptCid = 0;
            for (int i = 0; i < featureSlotCount; ++i) {
                featureData[i].valid = false;
            }  // 清空静态缓存
            cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, false);
            LOGI("[HCI] Disconnected reason=0x%02X, start inquiry", reason);
            gap_inquiry_start(30);
            break;
        }
        default:
            break;
    }
}

static void __not_in_flash_func(l2capPacketHandler)(const uint8_t packet_type, const uint16_t channel, uint8_t* packet, const uint16_t size) {
    if (packet_type == L2CAP_DATA_PACKET) {
        if (channel == hidInterruptCid) {
            if (size - 3 >= ds5StatePacketSize && packet[1] == 0x31) {
                union Ds5StateUnion* ds5StateUnion = (union Ds5StateUnion*)(packet + 3);
                setStatePacket(ds5StateUnion);

                // 静默检测
                if (ds5StateUnion->packet.LeftStickX < 120 || ds5StateUnion->packet.LeftStickX > 140 || ds5StateUnion->packet.LeftStickY < 120 || ds5StateUnion->packet.LeftStickY > 140 ||
                    ds5StateUnion->packet.RightStickX < 120 || ds5StateUnion->packet.RightStickX > 140 || ds5StateUnion->packet.RightStickY < 120 || ds5StateUnion->packet.RightStickY > 140 ||
                    ds5StateUnion->packet.TriggerLeft > 0 || ds5StateUnion->packet.TriggerRight > 0 || ds5StateUnion->data[7] != DirectionNone || ds5StateUnion->data[8] != 0x00 ||
                    ds5StateUnion->data[9] != 0x00) {
                    inactiveTime = get_absolute_time();
                } else if (absolute_time_diff_us(inactiveTime, get_absolute_time()) > (int64_t)(config.inactiveTime) * 60 * 1000 * 1000) {
                    LOGI("disconnect when inactive");
                    inactiveTime = get_absolute_time();
                    btDisconnect();
                }
            }
        } else if (channel == hidControlCid) {
            if (size < 1) {
                LOGW("[L2CAP] HID Control data empty");
                return;
            }
            if (checkDse) {
                // packet[0] == 0x02 只需 1 字节
                if (packet[0] == 0x02) {
                    LOGI("Connected DS5 Controller");
                    checkDse = false;
                    config.isDse = false;
                    tud_connect();
                } else if (size >= 2 && packet[0] == 0xA3 && packet[1] == 0x70) {
                    // 需要 2 字节才能读 packet[1]
                    LOGI("Connected DSE Controller");
                    checkDse = false;
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
        } else {
            LOGE("[L2CAP] Data on unknown channel 0x%04X (Interrupt: 0x%04X, Control: 0x%04X)", channel, hidInterruptCid, hidControlCid);
        }
        return;
    }

    const uint8_t eventType = hci_event_packet_get_type(packet);
    switch (eventType) {
        case L2CAP_EVENT_CHANNEL_OPENED: {
            const uint8_t status = l2cap_event_channel_opened_get_status(packet);
            const uint16_t localCid = l2cap_event_channel_opened_get_local_cid(packet);
            if (status == 0) {
                const uint16_t psm = l2cap_event_channel_opened_get_psm(packet);
                if (psm == PSM_HID_CONTROL) {
                    LOGI("[L2CAP] HID Control opened cid=0x%04X", localCid);
                    hidControlCid = localCid;

                    [[maybe_unused]] const uint16_t mtu = l2cap_get_remote_mtu_for_local_cid(hidControlCid);
                    LOGI("[L2CAP] Remote Control MTU: %d", mtu);
                } else if (psm == PSM_HID_INTERRUPT) {
                    LOGI("[L2CAP] HID Interrupt opened cid=0x%04X", localCid);
                    hidInterruptCid = localCid;
                    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, true);
                    inactiveTime = get_absolute_time();

                    LOGI("Init DualSense");
                    initFeature();
                    // 初始化手柄状态
                    uint8_t* controlBuffer = getBufferForSubPacket(subPacketTypeControl);
                    if (controlBuffer != nullptr) {
                        static_assert(subPacketControlSize == ds5ControlPacketSize, "subPacketControlSize != ds5ControlPacketSize");
                        memcpy(controlBuffer, ds5ControlInitPacket.data, subPacketControlSize);
                        writeSubPacket(controlBuffer, subPacketTypeControl);
                    } else {
                        LOGE("getBufferForSubPacket subPacketTypeControl");
                    }

                    [[maybe_unused]] const uint16_t mtu = l2cap_get_remote_mtu_for_local_cid(hidInterruptCid);
                    LOGI("[L2CAP] Remote Interrupt MTU: %d", mtu);

                    gap_connectable_control(0U);
                    gap_discoverable_control(0U);
                    // tud_connect();
                } else {
                    LOGE("[L2CAP] Unknown Channel psm: 0x%02X", psm);
                }

                /*if (hid_control_cid != 0 && hid_interrupt_cid != 0) {
                    LOGI("[L2CAP] HID channels ready, request CAN_SEND_NOW for SET_PROTOCOL");
                    l2cap_request_can_send_now_event(hid_control_cid);
                }*/
            } else {
                [[maybe_unused]] const uint16_t psm = l2cap_event_channel_opened_get_psm(packet);
                hidControlCid = 0;
                hidInterruptCid = 0;
                deviceFound = false;
                LOGI("[L2CAP] Open failed psm=0x%04X status=0x%02X", psm, status);
                btDisconnect();
            }
            break;
        }

        case L2CAP_EVENT_INCOMING_CONNECTION: {
            const uint16_t localCid = l2cap_event_incoming_connection_get_local_cid(packet);
            [[maybe_unused]] const uint16_t psm = l2cap_event_incoming_connection_get_psm(packet);
            LOGI("[L2CAP] Incoming connection psm=0x%04X cid=0x%04X", psm, localCid);
            l2cap_accept_connection(localCid);
            break;
        }

        case L2CAP_EVENT_CHANNEL_CLOSED: {
            const uint16_t localCid = l2cap_event_channel_closed_get_local_cid(packet);
            if (localCid == hidControlCid) {
                hidControlCid = 0;
                LOGI("[L2CAP] HID Control closed cid=0x%04X", localCid);
            } else if (localCid == hidInterruptCid) {
                hidInterruptCid = 0;
                LOGI("[L2CAP] HID Interrupt closed cid=0x%04X", localCid);
            } else {
                LOGI("[L2CAP] Channel closed cid=0x%04X", localCid);
            }
            if (hidControlCid == 0 && hidInterruptCid == 0) {
                btDisconnect();
            }
            break;
        }

        case L2CAP_EVENT_CAN_SEND_NOW: {
            {
                theRequestHasBeenSent = false;
                size_t len = 0;
                uint8_t* bluetoothRawPacket = getBluetoothRawPacket(&len);
                if (bluetoothRawPacket != nullptr) {
                    const uint8_t status = l2cap_send(hidInterruptCid, bluetoothRawPacket, len);
                    if (status != 0) {
                        LOGE("[L2CAP] L2CAP Send Error, Status: 0x%02X", status);
                    }
                    freeBluetoothRawPacket(bluetoothRawPacket);
                }

                btRequestSend();
            }
            break;
        }
        default:
            break;
    }
}

void __not_in_flash_func(btRequestSend)() {
    if (hidInterruptCid != 0 && !theRequestHasBeenSent && hasBluetoothRawPacketCanSend()) {
        theRequestHasBeenSent = true;
        l2cap_request_can_send_now_event(hidInterruptCid);
    }
}

uint16_t getFeatureData(const uint8_t reportId, uint8_t* outBuf, const uint16_t maxLen) {
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
        if (hidControlCid != 0) {
            const uint8_t getFeature[] = {0x43, reportId};
            l2cap_send(hidControlCid, getFeature, sizeof(getFeature));
            LOGD("[L2CAP] Requesting Get Feature Report 0x%02X", reportId);
        }
    }
    return copied;
}

void setFeatureData(const uint8_t reportId, const uint8_t* data, const uint16_t len) {
    if (hidControlCid != 0) {
        // 使用固定大小的静态缓冲替代主机长度可控的栈上 VLA，避免栈溢出
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
        l2cap_send(hidControlCid, getFeature, len + 2);

        LOGD("[L2CAP] Requesting Set Feature Report 0x%02X", reportId);
#if ENABLE_DEBUG
        printf_hexdump(getFeature, len + 2);
#endif
    }
}

void initFeature() {
    getFeatureData(0x09, nullptr, 0);
    getFeatureData(0x20, nullptr, 0);
    getFeatureData(0x22, nullptr, 0);
    getFeatureData(0x05, nullptr, 0);
    // DSE
    // check DSE by request 0x70 feature report. DSE return DEFAULT
    // If len == 1, it's DS5
    checkDse = true;
    getFeatureData(0x70, nullptr, 0);
}
