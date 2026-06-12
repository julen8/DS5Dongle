//
// Created by awalol on 2026/3/4.
//

#include <assert.h>
#include <bsp/board_api.h>
#include <hardware/clocks.h>
#include <hardware/vreg.h>
#include <hardware/watchdog.h>
#include <pico/critical_section.h>
#include <pico/cyw43_arch.h>
#include <stdint.h>
#include <stdio.h>

#include "audio.h"
#include "bluetoothPacket.h"
#include "bt.h"
#include "config.h"
#include "crc32.h"
#include "log.h"
#include "state.h"

void interruptLoop() {
    if (tud_hid_ready()) {
        if (!tud_hid_report(0x01, getStatePacket()->data, ds5StatePacketSize)) {
            LOGE("[USBHID] tud_hid_report error");
        }
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
                if (bufsize - 1 < subPacketControlSize) {
                    LOGE("Received control sub packet with size %d, expected %d", bufsize - 1, subPacketControlSize);
                    break;
                }

                uint8_t *controlBuffer = getBufferForSubPacket(subPacketTypeControl);
                if (controlBuffer != nullptr) {
                    memcpy(controlBuffer, buffer + 1, subPacketControlSize);
                    writeSubPacket(controlBuffer, subPacketTypeControl);
                } else {
                    LOGE("getBufferForSubPacket subPacketTypeControl");
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
    constexpr uint32_t sysClockKhz = 320000;
    set_sys_clock_khz(sysClockKhz, true);

    board_init();
    printf("\n\n===================\nBuild Time: " __DATE__ " " __TIME__ "\n===================\n\n");
    initCrc32();

    constexpr tusb_rhport_init_t devInit = {.role = TUSB_ROLE_DEVICE, .speed = TUSB_SPEED_FULL};
    tusb_init(BOARD_TUD_RHPORT, &devInit);
    tud_disconnect();
    board_init_after_tusb();

    const int ret = cyw43_arch_init();
    if (ret != 0) {
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
    bluetoothPacketInit();
    btInit();
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
