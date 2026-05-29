//
// Created by awalol on 2026/3/4.
//

#include <cstdio>

#include "audio.h"
#include "bsp/board_api.h"
#include "bt.h"
#include "config.h"
#include "hardware/clocks.h"
#include "hardware/vreg.h"
#include "hardware/watchdog.h"
#include "pico/cyw43_arch.h"
#include "utils.h"

// Pico SDK speciifically for waiting on conditions
#include "bluetoothPacket.h"
#include "log.h"
#include "pico/critical_section.h"
constexpr auto bluetoothInterruptDataSize = 63;
static uint8_t interrupt_in_data[] = {0x7f, 0x7d, 0x7f, 0x7e, 0x00, 0x00, 0xa7, 0x08, 0x00, 0x00, 0x00, 0x52, 0x43, 0x30, 0x41, 0x01, 0x00, 0x0e, 0x00, 0xef, 0xff,
                                      0x03, 0x03, 0x7b, 0x1b, 0x18, 0xf0, 0xcc, 0x9c, 0x60, 0x00, 0xfc, 0x80, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x09,
                                      0x09, 0x00, 0x00, 0x00, 0x00, 0x00, 0xa7, 0xad, 0x60, 0x00, 0x29, 0x18, 0x00, 0x53, 0x9f, 0x28, 0x35, 0xa5, 0xa8, 0x0c, 0x8b};
static_assert(sizeof(interrupt_in_data) == bluetoothInterruptDataSize, "interrupt_in_data must be 63 bytes long");

critical_section_t report_cs;
volatile bool report_dirty = false;

void interrupt_loop() {
    if (!tud_hid_ready()) return;

    // TODO: Refactor for better code reuse
    if (config.pollingRateMode != 2) {
        if (!tud_hid_report(0x01, interrupt_in_data, bluetoothInterruptDataSize)) {
            LOGE("[USBHID] tud_hid_report error");
        }
        return;
    }

    bool should_send = false;
    // Local buffer to hold the report data while we prepare it to send.
    uint8_t safe_report[bluetoothInterruptDataSize];

    critical_section_enter_blocking(&report_cs);
    if (report_dirty) {
        memcpy(safe_report, interrupt_in_data, bluetoothInterruptDataSize);
        report_dirty = false;
        should_send = true;
    }
    critical_section_exit(&report_cs);

    // Only send to TinyUSB if we actually grabbed fresh data
    if (should_send) {
        if (!tud_hid_report(0x01, safe_report, bluetoothInterruptDataSize)) {
            LOGE("[USBHID] tud_hid_report error");

            // If the report failed to queue, restore the dirty flag
            // so we try again on the next loop iteration.
            critical_section_enter_blocking(&report_cs);
            report_dirty = true;
            critical_section_exit(&report_cs);
        }
    }
}

void on_bt_data(CHANNEL_TYPE channel, uint8_t *data, uint16_t len) {
    // LOGD("[Main] BT data callback: channel=%u len=%u", channel, len);
    if (channel == INTERRUPT && data[1] == 0x31) {
        if ((data[56] & 1) != (interrupt_in_data[53] & 1)) {
            config.plugHeadset = (data[56] & 1) != 0;
        }

        if (len - 3 < bluetoothInterruptDataSize) {
            LOGW("(len - 3):%d < bluetoothInterruptDataSize:%d", len - 3, bluetoothInterruptDataSize);
            return;
        }

        if (config.pollingRateMode != 2) {
            memcpy(interrupt_in_data, data + 3, bluetoothInterruptDataSize);
            return;
        }

        // We add the critical section here to avoid any race conditions when writing to the interrupt_in_data buffer,
        // which is shared between the main loop and this callback.
        // The critical section ensures that only one thread can access the buffer at a time,
        // preventing data corruption and ensuring thread safety.
        // We also set the report_dirty flag to true to indicate that new data is available
        //  and needs to be sent in the next interrupt report.
        critical_section_enter_blocking(&report_cs);
        memcpy(interrupt_in_data, data + 3, bluetoothInterruptDataSize);
        report_dirty = true;
        critical_section_exit(&report_cs);
    }
}

// Invoked when received GET_REPORT control request
// Application must fill buffer report's content and return its length.
// Return zero will cause the stack to STALL request
uint16_t tud_hid_get_report_cb(uint8_t itf, uint8_t report_id, hid_report_type_t report_type, uint8_t *buffer, uint16_t reqlen) {
    (void)itf;
    (void)report_id;
    (void)report_type;
    (void)buffer;
    (void)reqlen;

    std::vector<uint8_t> feature_data = get_feature_data(report_id, reqlen);
    if (!feature_data.empty()) {
        memcpy(buffer, feature_data.data() + 1, feature_data.size() - 1);
    }

    return feature_data.empty() ? 0 : feature_data.size() - 1;
}

bool tud_audio_set_itf_cb(uint8_t rhport, tusb_control_request_t const *p_request) {
    (void)rhport;
    uint8_t const itf = tu_u16_low(p_request->wIndex);  // wInterface
    uint8_t const alt = tu_u16_low(p_request->wValue);  // bAlternateSetting

    if (itf == 1) {
        config.audioActive = (alt != 0);
        LOGI("[AUDIO] Set interface Speaker to alternate setting %d", alt);
    }

    return true;
}

// Invoked when received SET_REPORT control request or
// received data on OUT endpoint ( Report ID = 0, Type = 0 )
void tud_hid_set_report_cb(uint8_t itf, uint8_t report_id, hid_report_type_t report_type, uint8_t const *buffer, uint16_t bufsize) {
    // INTERRUPT OUT
    if (report_id == 0) {
        switch (buffer[0]) {
            case 0x02: {
                if (auto *statusBuffer = getBufferForSubPacket(subPacketType::status); statusBuffer != nullptr) {
                    const auto size = std::min(bufsize - 1, subPacketStatusSize);
                    memcpy(statusBuffer, buffer + 1, size);
                    if (size < subPacketStatusSize) {
                        memset(statusBuffer + size, 0, subPacketStatusSize - size);
                    }
                    writeSubPacket(statusBuffer, subPacketType::status);
                } else {
                    LOGE("getBufferForSubPacket subPacketType::status");
                }
                break;
            }
            default:
                LOGE("Unknown sub packet type:%02X", buffer[0]);
                break;
        }
    }
    if (report_id == 0x80 ||
        // DSE: Write Profile Block
        report_id == 0x60 || report_id == 0x62 || report_id == 0x61) {
        set_feature_data(report_id, const_cast<uint8_t *>(buffer), bufsize);
        return;
    }
}

int main() {
    vreg_set_voltage(VREG_VOLTAGE_1_20);
    sleep_ms(1000);
    set_sys_clock_khz(SYS_CLOCK_KHZ, true);

    board_init();
    constexpr tusb_rhport_init_t devInit = {.role = TUSB_ROLE_DEVICE, .speed = TUSB_SPEED_FULL};
    tusb_init(BOARD_TUD_RHPORT, &devInit);
    tud_disconnect();
    board_init_after_tusb();

    if (const auto ret = cyw43_arch_init(); ret != 0) {
        LOGE("Failed to initialize CYW43:%d", ret);
        return 1;
    }
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, false);

    if (watchdog_caused_reboot()) {
        LOGW("Rebooted by Watchdog!");
        // 当崩溃重启以后，闪三下灯
        for (int i = 0; i < 6; i++) {
            if (i % 2 == 0) {
                cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, true);
            } else {
                cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, false);
            }
            sleep_ms(500);
        }
    } else {
        LOGI("Clean boot");
    }

    // Initialize the critical section for the report buffer
    critical_section_init(&report_cs);
    bluetoothPacketInit();
    bt_init();
    bt_register_data_callback(on_bt_data);
    audioInit();
    watchdog_enable(5000, true);

    for (;;) {
        watchdog_update();
        cyw43_arch_poll();
        tud_task();
        audioLoop();
        interrupt_loop();
    }
}
