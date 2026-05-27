//
// Created by awalol on 2026/3/4.
//

#include "bt.h"

#include <bsp/board_api.h>
#include <btstack_event.h>
#include <classic/sdp_server.h>
#include <l2cap.h>
#include <pico/cyw43_arch.h>
#include <pico/util/queue.h>

#include <cstdio>
#include <cstring>
#include <unordered_map>
#include <vector>

#include "bluetoothPacket.h"
#include "config.h"
#include "log.h"
#include "utils.h"

#define MTU_CONTROL 672
#define MTU_INTERRUPT 672

static void hci_packet_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size);
static void l2cap_packet_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size);

static btstack_packet_callback_registration_t hci_event_callback_registration, l2cap_event_callback_registration;
static bd_addr_t current_device_addr;
static bool device_found = false;
static bool new_pair = false;  // 只有新匹配的设备才用创建channel，自动重连走的是service
static hci_con_handle_t acl_handle = HCI_CON_HANDLE_INVALID;
static uint16_t hid_control_cid;
static uint16_t hid_interrupt_cid;
static bt_data_callback_t bt_data_callback = nullptr;
static bool check_dse = false;
static bool theRequestHasBeenSent = false;
std::unordered_map<uint8_t, std::vector<uint8_t> > feature_data;
absolute_time_t inactive_time = 0;  // 手柄长时间静默

void bt_register_data_callback(bt_data_callback_t callback) { bt_data_callback = callback; }

bool bt_disconnect() {
    if (acl_handle == HCI_CON_HANDLE_INVALID) {
        return false;
    }

    // 0x13 = remote user terminated connection
    hci_send_cmd(&hci_disconnect, acl_handle, 0x13);
    return true;
}

void bt_l2cap_init() {
    l2cap_event_callback_registration.callback = &l2cap_packet_handler;
    l2cap_add_event_handler(&l2cap_event_callback_registration);
    // 修复重连后自动断开的关键点
    sdp_init();
    l2cap_register_service(l2cap_packet_handler, PSM_HID_CONTROL, MTU_CONTROL, LEVEL_2);
    l2cap_register_service(l2cap_packet_handler, PSM_HID_INTERRUPT, MTU_INTERRUPT, LEVEL_2);

    l2cap_init();
}

int bt_init() {
    bt_l2cap_init();

    // SSP (Secure Simple Pairing)
    gap_ssp_set_enable(true);
    gap_secure_connections_enable(true);
    gap_ssp_set_io_capability(SSP_IO_CAPABILITY_DISPLAY_YES_NO);
    gap_ssp_set_authentication_requirement(SSP_IO_AUTHREQ_MITM_PROTECTION_NOT_REQUIRED_GENERAL_BONDING);

    gap_connectable_control(1);
    gap_discoverable_control(1);

    hci_event_callback_registration.callback = &hci_packet_handler;
    hci_add_event_handler(&hci_event_callback_registration);

    hci_power_control(HCI_POWER_ON);
    return 0;
}

static void hci_packet_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size) {
    const uint8_t event_type = hci_event_packet_get_type(packet);

    switch (event_type) {
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
            uint32_t cod;

            if (event_type == HCI_EVENT_INQUIRY_RESULT) {
                cod = hci_event_inquiry_result_get_class_of_device(packet);
                hci_event_inquiry_result_get_bd_addr(packet, addr);
            } else if (event_type == HCI_EVENT_INQUIRY_RESULT_WITH_RSSI) {
                cod = hci_event_inquiry_result_with_rssi_get_class_of_device(packet);
                hci_event_inquiry_result_with_rssi_get_bd_addr(packet, addr);
            } else {
                cod = hci_event_extended_inquiry_response_get_class_of_device(packet);
                hci_event_extended_inquiry_response_get_bd_addr(packet, addr);
            }

            // CoD 0x002508 = Gamepad (Major: Peripheral, Minor: Gamepad)
            if ((cod & 0x000F00) == 0x000500) {
                LOGI("[HCI] Gamepad found: %s (CoD: 0x%06x)", bd_addr_to_str(addr), (unsigned int)cod);
                bd_addr_copy(current_device_addr, addr);
                device_found = true;
                gap_inquiry_stop();
            }
            break;
        }

        case GAP_EVENT_INQUIRY_COMPLETE:
        case HCI_EVENT_INQUIRY_COMPLETE: {
            LOGI("[HCI] Inquiry complete.");
            if (device_found) {
                LOGI("[HCI] Connecting to %s...", bd_addr_to_str(current_device_addr));
                new_pair = true;
                hci_send_cmd(&hci_create_connection, current_device_addr, hci_usable_acl_packet_types(), 0, 0, 0, 1);
                break;
            }
            if (event_type == HCI_EVENT_INQUIRY_COMPLETE) {
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
            LOGI("[HCI] CmdStatus %s(0x%04X) status=0x%02X", opcode_to_str(opcode), opcode, status);
            if (opcode == HCI_OPCODE_HCI_CREATE_CONNECTION && status != ERROR_CODE_SUCCESS) {
                device_found = false;
                new_pair = false;
                LOGW("[HCI] Create connection rejected, restart inquiry");
                // gap_inquiry_start(30);
            }
            break;
        }

        case HCI_EVENT_COMMAND_COMPLETE: {
            const uint8_t status = hci_event_command_complete_get_return_parameters(packet)[0];
            const uint16_t opcode = hci_event_command_complete_get_command_opcode(packet);
            LOGI("[HCI] CmdComplete %s(0x%04X) status=0x%02X", opcode_to_str(opcode), opcode, status);
            break;
        }

        case HCI_EVENT_CONNECTION_COMPLETE: {
            const uint8_t status = hci_event_connection_complete_get_status(packet);
            if (status == 0) {
                const hci_con_handle_t handle = hci_event_connection_complete_get_connection_handle(packet);
                acl_handle = handle;
                hci_event_connection_complete_get_bd_addr(packet, current_device_addr);
                LOGI("[HCI] ACL connected handle=0x%04X", handle);
                LOGI("[HCI] Request authentication on handle=0x%04X", handle);
                hci_send_cmd(&hci_authentication_requested, handle);
            } else {
                device_found = false;
                new_pair = false;
                LOGW("[HCI] ACL connect failed status=0x%02X, restart inquiry", status);
                // gap_inquiry_start(30);
            }
            break;
        }

        case HCI_EVENT_LINK_KEY_REQUEST: {
            bd_addr_t addr;
            hci_event_link_key_request_get_bd_addr(packet, addr);
            link_key_t link_key;
            link_key_type_t link_key_type;
            bool link = gap_get_link_key_for_bd_addr(addr, link_key, &link_key_type);
            LOGI("[HCI] Link key: ");
            for (int i = 0; i < sizeof(link_key_t); i++) {
                LOGI("%02X", link_key[i]);
            }
            if (link) {
                LOGI("[HCI] Link key request from %s, reply stored key type=%u", bd_addr_to_str(addr), (unsigned int)link_key_type);
                hci_send_cmd(&hci_link_key_request_reply, addr, link_key);
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
                LOGW("[HCI] Authentication failed, drop stored key for %s", bd_addr_to_str(current_device_addr));
                gap_drop_link_key_for_bd_addr(current_device_addr);
                // gap_inquiry_start(30);
            } else {
                hci_send_cmd(&hci_set_connection_encryption, handle, 1);
            }
            break;
        }

        case HCI_EVENT_ENCRYPTION_CHANGE: {
            const uint8_t status = hci_event_encryption_change_get_status(packet);
            const hci_con_handle_t handle = hci_event_encryption_change_get_connection_handle(packet);
            const uint8_t enabled = hci_event_encryption_change_get_encryption_enabled(packet);
            LOGI("[HCI] Encryption change handle=0x%04X status=0x%02X enabled=%u", handle, status, enabled);
            if (status == ERROR_CODE_SUCCESS && enabled) {
                LOGI("[L2CAP] Open HID channels");
                if (new_pair) {
                    if (hid_control_cid == 0) {
                        l2cap_create_channel(l2cap_packet_handler, current_device_addr, PSM_HID_CONTROL, MTU_CONTROL, &hid_control_cid);
                    } else if (hid_interrupt_cid == 0) {
                        l2cap_create_channel(l2cap_packet_handler, current_device_addr, PSM_HID_INTERRUPT, MTU_INTERRUPT, &hid_interrupt_cid);
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
                bd_addr_copy(current_device_addr, addr);
                gap_inquiry_stop();
                hci_send_cmd(&hci_accept_connection_request, addr, 0x01);
            }
            break;
        }

        case HCI_EVENT_DISCONNECTION_COMPLETE: {
            tud_disconnect();
            gap_connectable_control(1);
            gap_discoverable_control(1);
            const uint8_t reason = hci_event_disconnection_complete_get_reason(packet);
            device_found = false;
            new_pair = false;
            acl_handle = HCI_CON_HANDLE_INVALID;
            hid_control_cid = 0;
            hid_interrupt_cid = 0;
            feature_data.clear();
            cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, false);
            LOGI("[HCI] Disconnected reason=0x%02X, start inquiry", reason);
            gap_inquiry_start(30);
            break;
        }
    }
}

static void l2cap_packet_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size) {
    (void)channel;

    if (packet_type == L2CAP_DATA_PACKET) {
        if (channel == hid_interrupt_cid) {
            bt_data_callback(INTERRUPT, packet, size);

            // 静默检测
            if (packet[3] < 120 || packet[3] > 140 || packet[4] < 120 || packet[4] > 140 || packet[5] < 120 || packet[5] > 140 || packet[6] < 120 || packet[6] > 140 || packet[7] > 0 ||
                packet[8] > 0 || packet[10] != 0x08 || packet[11] != 0x00 || packet[12] != 0x00) {
                inactive_time = get_absolute_time();
            } else if (absolute_time_diff_us(inactive_time, get_absolute_time()) > static_cast<int64_t>(config.inactiveTime) * 60 * 1000 * 1000) {
                LOGI("disconnect when inactive");
                inactive_time = get_absolute_time();
                bt_disconnect();
            }
        } else if (channel == hid_control_cid) {
            if (check_dse) {
                if (packet[0] == 0xA3 && packet[1] == 0x70) {
                    LOGI("Connected DSE Controller");
                    check_dse = false;
                    config.isDse = true;
                    tud_connect();
                } else if (packet[0] == 0x02) {
                    LOGI("Connected DS5 Controller");
                    check_dse = false;
                    config.isDse = false;
                    tud_connect();
                }
            }
            if (packet[0] == 0xA3) {
                uint8_t report_id = packet[1];
                feature_data[report_id].assign(packet + 1, packet + size);
                LOGD("[L2CAP] Stored Feature Report 0x%02X, len=%u", report_id, size - 1);
            }

            LOGD("[L2CAP] HID Control data len=%u", size);
#if ENABLE_DEBUG
            printf_hexdump(packet, size);
#endif

            bt_data_callback(CONTROL, packet, size);
        } else {
            LOGE("[L2CAP] Data on unknown channel 0x%04X (Interrupt: 0x%04X, Control: 0x%04X)", channel, hid_interrupt_cid, hid_control_cid);
        }
        return;
    }

    const uint8_t event_type = hci_event_packet_get_type(packet);
    switch (event_type) {
        case L2CAP_EVENT_CHANNEL_OPENED: {
            const uint8_t status = l2cap_event_channel_opened_get_status(packet);
            const uint16_t local_cid = l2cap_event_channel_opened_get_local_cid(packet);
            if (status == 0) {
                const uint16_t psm = l2cap_event_channel_opened_get_psm(packet);
                if (psm == PSM_HID_CONTROL) {
                    LOGI("[L2CAP] HID Control opened cid=0x%04X", local_cid);
                    hid_control_cid = local_cid;

                    const auto mtu = l2cap_get_remote_mtu_for_local_cid(hid_control_cid);
                    LOGI("[L2CAP] Remote Control MTU: %d", mtu);
                } else if (psm == PSM_HID_INTERRUPT) {
                    LOGI("[L2CAP] HID Interrupt opened cid=0x%04X", local_cid);
                    hid_interrupt_cid = local_cid;
                    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, true);
                    inactive_time = get_absolute_time();

                    LOGI("Init DualSense");
                    init_feature();
                    // 初始化手柄状态
                    if (auto *statusBuffer = getBufferForSubPacket(subPacketType::status); statusBuffer != nullptr) {
                        memcpy(statusBuffer, stateInitData, subPacketStatusSize);
                        writeSubPacket(statusBuffer, subPacketType::status);
                    } else {
                        LOGE("getBufferForSubPacket subPacketType::status");
                    }

                    const auto mtu = l2cap_get_remote_mtu_for_local_cid(hid_interrupt_cid);
                    LOGI("[L2CAP] Remote Interrupt MTU: %d", mtu);

                    gap_connectable_control(false);
                    gap_discoverable_control(false);
                    // tud_connect();
                } else {
                    LOGE("[L2CAP] Unknown Channel psm: 0x%02X", psm);
                }

                /*if (hid_control_cid != 0 && hid_interrupt_cid != 0) {
                    LOGI("[L2CAP] HID channels ready, request CAN_SEND_NOW for SET_PROTOCOL");
                    l2cap_request_can_send_now_event(hid_control_cid);
                }*/
            } else {
                const uint16_t psm = l2cap_event_channel_opened_get_psm(packet);
                hid_control_cid = 0;
                hid_interrupt_cid = 0;
                device_found = false;
                LOGI("[L2CAP] Open failed psm=0x%04X status=0x%02X", psm, status);
                bt_disconnect();
            }
            break;
        }

        case L2CAP_EVENT_INCOMING_CONNECTION: {
            const uint16_t local_cid = l2cap_event_incoming_connection_get_local_cid(packet);
            const uint16_t psm = l2cap_event_incoming_connection_get_psm(packet);
            LOGI("[L2CAP] Incoming connection psm=0x%04X cid=0x%04X", psm, local_cid);
            l2cap_accept_connection(local_cid);
            break;
        }

        case L2CAP_EVENT_CHANNEL_CLOSED: {
            const uint16_t local_cid = l2cap_event_channel_closed_get_local_cid(packet);
            if (local_cid == hid_control_cid) {
                hid_control_cid = 0;
                LOGI("[L2CAP] HID Control closed cid=0x%04X", local_cid);
            } else if (local_cid == hid_interrupt_cid) {
                hid_interrupt_cid = 0;
                LOGI("[L2CAP] HID Interrupt closed cid=0x%04X", local_cid);
            } else {
                LOGI("[L2CAP] Channel closed cid=0x%04X", local_cid);
            }
            if (hid_control_cid == 0 && hid_interrupt_cid == 0) {
                bt_disconnect();
            }
            break;
        }

        case L2CAP_EVENT_CAN_SEND_NOW: {
            {
                theRequestHasBeenSent = false;
                size_t size = 0;
                auto *bluetoothRawPacket = getBluetoothRawPacket(&size);
                if (bluetoothRawPacket != nullptr) {
                    if (const auto status = l2cap_send(hid_interrupt_cid, bluetoothRawPacket, size); status != 0) {
                        LOGE("[L2CAP] L2CAP Send Error, Status: 0x%02X", status);
                    }
                    freeBluetoothRawPacket(bluetoothRawPacket);
                }

                btRequestSend();
            }
            break;
        }
    }
}

void btRequestSend() {
    if (hid_interrupt_cid != 0 && !theRequestHasBeenSent && hasBluetoothRawPacketCanSend()) {
        theRequestHasBeenSent = true;
        l2cap_request_can_send_now_event(hid_interrupt_cid);
    }
}

std::vector<uint8_t> get_feature_data(uint8_t reportId, uint16_t len) {
    // 若为0x81则会请求新内容，其他若有旧数据则不进行请求
    auto ret = std::vector<uint8_t>{};
    if (feature_data.contains(reportId)) {
        ret = feature_data[reportId];
    }

    if (!feature_data.contains(reportId) ||
        // Get Test Command Result
        reportId == 0x81 ||
        // DSE: Set Profile Save?
        reportId == 0x63 || reportId == 0x65 || reportId == 0x64) {
        if (hid_control_cid != 0) {
            uint8_t get_feature[] = {0x43, reportId};
            l2cap_send(hid_control_cid, get_feature, sizeof(get_feature));
            LOGD("[L2CAP] Requesting Get Feature Report 0x%02X", reportId);
        }
    }
    return ret;
}

void set_feature_data(uint8_t reportId, uint8_t *data, uint16_t len) {
    if (hid_control_cid != 0) {
        uint8_t get_feature[len + 2];
        get_feature[0] = 0x53;
        get_feature[1] = reportId;
        memcpy(get_feature + 2, data, len);
        fill_feature_report_checksum(get_feature + 1, len + 1);
        l2cap_send(hid_control_cid, get_feature, len + 2);

        LOGD("[L2CAP] Requesting Set Feature Report 0x%02X", reportId);
#if ENABLE_DEBUG
        printf_hexdump(get_feature, len + 2);
#endif
    }
}

void init_feature() {
    get_feature_data(0x09, 20);
    get_feature_data(0x20, 64);
    get_feature_data(0x22, 64);
    get_feature_data(0x05, 41);
    // DSE
    // check DSE by request 0x70 feature report. DSE return DEFAULT
    // If len == 1, it's DS5
    check_dse = true;
    get_feature_data(0x70, 64);
}
