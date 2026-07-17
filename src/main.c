//
// Created by awalol on 2026/3/4.
//

#include <bsp/board_api.h>
#include <hardware/clocks.h>
#include <hardware/watchdog.h>
#include <pico/cyw43_arch.h>
#include <stdio.h>

#include "audio.h"
#include "bluetooth_packet.h"
#include "bt.h"
#include "config.h"
#include "crc32.h"
#include "log.h"
#include "usb.h"

int main() {
    set_sys_clock_khz(SYS_CLOCK_KHZ, true);
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

    bluetoothPacketInit();
    btInit();
    if (!audioInit()) {
        LOGE("Audio initialization failed");
        return 1;
    }
    watchdog_enable(1000, true);

    for (;;) {
        watchdog_update();
        if (config.audioActive && config.pollingRateMode == 2) {
            // 1000Hz模式下，此时蓝牙的发包会比较费时，所以在这里多发送一次usb数据，保证usb的回报率能保持到1000Hz
            tud_task();
            usbInterruptLoop();
        }
        cyw43_arch_poll();
        tud_task();
        usbInterruptLoop();
        audioLoop();
        btRequestSend();
        btInquiringLed();
    }
}
