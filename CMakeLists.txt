cmake_minimum_required(VERSION 3.26)

set(CMAKE_C_STANDARD 23)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(PICO_BOARD pico2_w CACHE STRING "Board type")

#set(PICO_CYW43_SUPPORTED 1)
#set(PICO_CYW43_ARCH_THREADSAFE_BACKGROUND 1)


# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(ds5-bridge C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()


add_executable(ds5-bridge
        src/usb_descriptors.c
        src/main.c
        src/usb.c
        src/audio.c
        src/bt.c
        src/config.c
        src/bluetoothPacket.c
        src/crc32.c
        src/lerp_resampler.c
        src/state.c
)

target_include_directories(ds5-bridge PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}/inc
)

# Opus
set(OPUS_FIXED_POINT ON)
set(HAVE_LRINTF ON)
set(OPUS_ENABLE_FLOAT_API OFF)
add_subdirectory(lib/opus)

target_compile_options(ds5-bridge PRIVATE -fsigned-char -Wall -O3 -Wdouble-promotion -mfloat-abi=softfp)
target_compile_options(opus PRIVATE -O3 -mfloat-abi=softfp)

target_compile_definitions(ds5-bridge PRIVATE
        CYW43_LWIP=0
        PICO_FLASH_ASSUME_CORE1_SAFE=1
        CYW43_PIO_CLOCK_DIV_INT=5
)

pico_set_program_name(ds5-bridge "ds5-bridge")

pico_enable_stdio_uart(ds5-bridge 1)
pico_enable_stdio_usb(ds5-bridge 0)

target_link_libraries(ds5-bridge
        pico_stdlib
        pico_multicore
        hardware_watchdog
        hardware_timer
        hardware_flash
        pico_btstack_classic
        pico_cyw43_arch_poll
        pico_btstack_cyw43
        tinyusb_device
        tinyusb_board
        opus
)

pico_add_extra_outputs(ds5-bridge)
