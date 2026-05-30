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
static uint8_t interruptInData[] = {0x7f, 0x7d, 0x7f, 0x7e, 0x00, 0x00, 0xa7, 0x08, 0x00, 0x00, 0x00, 0x52, 0x43, 0x30, 0x41, 0x01, 0x00, 0x0e, 0x00, 0xef, 0xff,
                                    0x03, 0x03, 0x7b, 0x1b, 0x18, 0xf0, 0xcc, 0x9c, 0x60, 0x00, 0xfc, 0x80, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x09,
                                    0x09, 0x00, 0x00, 0x00, 0x00, 0x00, 0xa7, 0xad, 0x60, 0x00, 0x29, 0x18, 0x00, 0x53, 0x9f, 0x28, 0x35, 0xa5, 0xa8, 0x0c, 0x8b};
static_assert(sizeof(interruptInData) == bluetoothInterruptDataSize, "interrupt_in_data must be 63 bytes long");

critical_section_t reportCs;
volatile bool reportDirty = false;

void interruptLoop() {
    if (!tud_hid_ready()) {
        return;
    }

    // TODO: Refactor for better code reuse
    if (config.pollingRateMode != 2) {
        if (!tud_hid_report(0x01, interruptInData, bluetoothInterruptDataSize)) {
            LOGE("[USBHID] tud_hid_report error");
        }
        return;
    }

    bool shouldSend = false;
    // Local buffer to hold the report data while we prepare it to send.
    uint8_t safeReport[bluetoothInterruptDataSize];

    critical_section_enter_blocking(&reportCs);
    if (reportDirty) {
        memcpy(safeReport, interruptInData, bluetoothInterruptDataSize);
        reportDirty = false;
        shouldSend = true;
    }
    critical_section_exit(&reportCs);

    // Only send to TinyUSB if we actually grabbed fresh data
    if (shouldSend) {
        if (!tud_hid_report(0x01, safeReport, bluetoothInterruptDataSize)) {
            LOGE("[USBHID] tud_hid_report error");

            // If the report failed to queue, restore the dirty flag
            // so we try again on the next loop iteration.
            critical_section_enter_blocking(&reportCs);
            reportDirty = true;
            critical_section_exit(&reportCs);
        }
    }
}

void onBtData(const CHANNEL_TYPE channel, const uint8_t *data, const uint16_t len) {
    // LOGD("[Main] BT data callback: channel=%u len=%u", channel, len);
    // 先做长度/边界校验，避免短包导致越界读
    if (channel == INTERRUPT && len >= 2 && data[1] == 0x31) {
        if (len - 3 < bluetoothInterruptDataSize) {
            LOGW("(len - 3):%d < bluetoothInterruptDataSize:%d", len - 3, bluetoothInterruptDataSize);
            return;
        }

        // 此时 len >= bluetoothInterruptDataSize + 3 (>=66)，data[56]/interrupt_in_data[53] 均在范围内
        if ((data[56] & 1) != (interruptInData[53] & 1)) {
            config.plugHeadset = (data[56] & 1) != 0;
        }

        if (config.pollingRateMode != 2) {
            memcpy(interruptInData, data + 3, bluetoothInterruptDataSize);
            return;
        }

        // We add the critical section here to avoid any race conditions when writing to the interrupt_in_data buffer,
        // which is shared between the main loop and this callback.
        // The critical section ensures that only one thread can access the buffer at a time,
        // preventing data corruption and ensuring thread safety.
        // We also set the report_dirty flag to true to indicate that new data is available
        //  and needs to be sent in the next interrupt report.
        critical_section_enter_blocking(&reportCs);
        memcpy(interruptInData, data + 3, bluetoothInterruptDataSize);
        reportDirty = true;
        critical_section_exit(&reportCs);
    }
}

// Invoked when received GET_REPORT control request
// Application must fill buffer report's content and return its length.
// Return zero will cause the stack to STALL request
uint16_t tud_hid_get_report_cb(uint8_t itf, uint8_t report_id, hid_report_type_t report_type, uint8_t *buffer, uint16_t reqlen) {
    // 直接写入 TinyUSB 提供的 buffer，避免每次 GET_REPORT 都构造临时 std::vector 造成堆分配/拷贝
    return getFeatureData(report_id, buffer, reqlen);
}

bool tud_audio_set_itf_cb(uint8_t rhport, tusb_control_request_t const *p_request) {
    uint8_t const itf = tu_u16_low(p_request->wIndex);  // wInterface
    uint8_t const alt = tu_u16_low(p_request->wValue);  // bAlternateSetting

    if (itf == 1) {
        config.audioActive = (alt != 0);
        LOGD("[AUDIO] Set interface Speaker to alternate setting %d", alt);
    }

    return true;
}

// Invoked when received SET_REPORT control request or
// received data on OUT endpoint ( Report ID = 0, Type = 0 )
void tud_hid_set_report_cb(uint8_t itf, uint8_t report_id, hid_report_type_t report_type, uint8_t const *buffer, uint16_t bufsize) {
    // INTERRUPT OUT
    if (report_id == 0) {
        if (bufsize < 1) {
            return;
        }
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
        setFeatureData(report_id, buffer, bufsize);
        return;
    }
}

int main() {
    vreg_set_voltage(VREG_VOLTAGE_1_20);
    sleep_ms(1000);
    set_sys_clock_khz(SYS_CLOCK_KHZ, true);

    board_init();
    printf("\n\n===================\nBuild Time: " __DATE__ " " __TIME__ "\n===================\n\n");

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
    critical_section_init(&reportCs);
    bluetoothPacketInit();
    btInit();
    btRegisterDataCallback(onBtData);
    audioInit();
    watchdog_enable(5000, true);

    for (;;) {
        watchdog_update();
        cyw43_arch_poll();
        interruptLoop();
        tud_task();
        audioLoop();
        interruptLoop();
    }
}
