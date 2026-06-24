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
#include <stdio.h>

#include "audio.h"
#include "bluetooth_packet.h"
#include "bt.h"
#include "crc32.h"
#include "log.h"
#include "usb.h"

int main() {
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
    audioInit();
    watchdog_enable(5000, true);

    for (;;) {
        watchdog_update();
        cyw43_arch_poll();
        tud_task();
        usbInterruptLoop();
        audioLoop();
        btInquiringLed();
    }
}
