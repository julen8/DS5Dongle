cmake_minimum_required(VERSION 3.26)

set(CMAKE_C_STANDARD 23)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(PICO_BOARD pico2_w CACHE STRING "Board type")

#set(PICO_CYW43_SUPPORTED 1)
#set(PICO_CYW43_ARCH_THREADSAFE_BACKGROUND 1)


# Pull in Raspberry Pi Pico SDK (must be before project)
include(cmake/pico_sdk_import.cmake)

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
        src/haptic_resampler.c
#        src/ram_mem.c
)

target_include_directories(ds5-bridge PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}/inc
)

# Opus
set(OPUS_FIXED_POINT ON)
set(HAVE_LRINTF ON)
set(OPUS_ENABLE_FLOAT_API OFF)
add_subdirectory(lib/opus)

# Phase A.3: Move libopus to RAM via .time_critical sections.
# libopus is built by add_subdirectory(lib/opus); the target `opus` is defined in
# the subdirectory's CMakeLists.txt, so we can't use POST_BUILD here. Instead we
# use a stamp-file custom command + custom target that runs after opus is built
# and before ds5-bridge links. Renames .text and .rodata of libopus.a to
# .time_critical.* which the Pico SDK linker places in RAM (loaded from flash
# at boot, executed from RAM at runtime). ~220 KB total relocated, ~5-6 ms boot
# time impact for the flash-to-RAM copy.
add_custom_command(
        OUTPUT ${CMAKE_BINARY_DIR}/opus_ram_relocate.stamp
        COMMAND ${CMAKE_OBJCOPY} --rename-section .text=.time_critical.opus_text --rename-section .rodata=.time_critical.opus_rodata --rename-section .rodata.str1.4=.time_critical.opus_strings $<TARGET_FILE:opus>
        COMMAND ${CMAKE_COMMAND} -E touch ${CMAKE_BINARY_DIR}/opus_ram_relocate.stamp
        DEPENDS $<TARGET_FILE:opus>
        COMMENT "Phase A.3: relocating libopus to .time_critical.* sections"
        VERBATIM
)
add_custom_target(opus_ram_relocate ALL DEPENDS ${CMAKE_BINARY_DIR}/opus_ram_relocate.stamp)
add_dependencies(ds5-bridge opus_ram_relocate)

# Phases B.2-B.4: Move per-packet BT/USB hot-path functions that live in pristine
# SDK/submodule sources (btstack, cyw43-driver, tinyusb, pico_util) into RAM. They
# compile directly into ds5-bridge and can't take __not_in_flash_func at the source.
# Since -ffunction-sections is on, each function has its own .text.<name> section;
# relocate_to_ram() renames the hot ones to .time_critical.<name> on the compiled
# objects in a PRE_LINK step (after compile, before link), and the Pico SDK linker
# places .time_critical.* in RAM (LMA flash, VMA RAM). The helper locates each
# object by its stable SDK-relative path SUFFIX, because the object's path under the
# target's .dir differs by environment (in-tree pico-sdk in CI vs out-of-tree
# locally). RENAMES are |-separated old=new pairs; objcopy skips an absent section,
# so this is idempotent across incremental relinks.
function(relocate_to_ram _suffix _renames _comment)
    add_custom_command(TARGET ds5-bridge PRE_LINK
            COMMAND ${CMAKE_COMMAND}
            "-DOBJROOT=${CMAKE_BINARY_DIR}/CMakeFiles/ds5-bridge.dir"
            "-DOBJCOPY=${CMAKE_OBJCOPY}"
            "-DSUFFIX=${_suffix}"
            "-DRENAMES=${_renames}"
            -P "${CMAKE_SOURCE_DIR}/cmake/relocate_to_ram.cmake"
            COMMENT "${_comment}"
            VERBATIM)
endfunction()

# --- Phase B.2: Tier-1 -- per-packet BT receive + USB FIFO data path ---
relocate_to_ram(lib/btstack/src/hci.c.o
        ".text.packet_handler=.time_critical.bt_packet_handler"
        "Phase B.2: hci.c packet_handler -> RAM")
relocate_to_ram(lib/btstack/src/l2cap.c.o
        ".text.l2cap_acl_handler=.time_critical.l2cap_acl_handler"
        "Phase B.2: l2cap.c l2cap_acl_handler -> RAM")
relocate_to_ram(lib/tinyusb/src/common/tusb_fifo.c.o
        ".text.tu_fifo_read=.time_critical.tu_fifo_read|.text.tu_fifo_read_n=.time_critical.tu_fifo_read_n|.text.tu_fifo_write=.time_critical.tu_fifo_write|.text.tu_fifo_write_n=.time_critical.tu_fifo_write_n|.text.tu_fifo_count=.time_critical.tu_fifo_count"
        "Phase B.2: tusb_fifo.c data path -> RAM")
relocate_to_ram(src/rp2_common/pico_cyw43_driver/cyw43_bus_pio_spi.c.o
        ".text.cyw43_spi_transfer=.time_critical.cyw43_spi_transfer|.text.cyw43_read_bytes=.time_critical.cyw43_read_bytes|.text.cyw43_write_bytes=.time_critical.cyw43_write_bytes|.text.cyw43_read_reg_u32=.time_critical.cyw43_read_reg_u32|.text.cyw43_read_reg_u16=.time_critical.cyw43_read_reg_u16|.text.cyw43_read_reg_u8=.time_critical.cyw43_read_reg_u8|.text.cyw43_write_reg_u32=.time_critical.cyw43_write_reg_u32|.text.cyw43_write_reg_u16=.time_critical.cyw43_write_reg_u16|.text.cyw43_write_reg_u8=.time_critical.cyw43_write_reg_u8|.text.read_reg_u32_swap=.time_critical.read_reg_u32_swap|.text.write_reg_u32_swap=.time_critical.write_reg_u32_swap|.text.ns_delay.constprop.0=.time_critical.cyw43_ns_delay"
        "Phase B.2: cyw43 PIO-SPI transport -> RAM")
relocate_to_ram(lib/cyw43-driver/src/cyw43_ll.c.o
        ".text.cyw43_ll_sdpcm_poll_device=.time_critical.cyw43_ll_sdpcm_poll_device|.text.cyw43_ll_process_packets=.time_critical.cyw43_ll_process_packets|.text.cyw43_sdpcm_send_common=.time_critical.cyw43_sdpcm_send_common|.text.cyw43_do_ioctl.part.0=.time_critical.cyw43_do_ioctl|.text.cyw43_ll_bt_has_work=.time_critical.cyw43_ll_bt_has_work|.text.cyw43_ll_parse_async_event=.time_critical.cyw43_ll_parse_async_event"
        "Phase B.2: cyw43 SDPCM pump -> RAM")
relocate_to_ram(lib/cyw43-driver/src/cyw43_ctrl.c.o
        ".text.cyw43_poll_func=.time_critical.cyw43_poll_func|.text.cyw43_bluetooth_hci_read=.time_critical.cyw43_bluetooth_hci_read|.text.cyw43_bluetooth_hci_write=.time_critical.cyw43_bluetooth_hci_write"
        "Phase B.2: cyw43 poll + BT HCI bus -> RAM")

# --- Phase B.3: Tier-2 -- run-loop TX pumps + per-transfer submit paths ---
# state_update is intentionally NOT moved: it runs on the host->controller HID-OUT
# (rumble/LED) path, not the audio frame path, so it stays in flash for headroom.
relocate_to_ram(lib/btstack/src/hci.c.o
        ".text.hci_run=.time_critical.hci_run|.text.hci_send_acl_packet_fragments=.time_critical.hci_send_acl_packet_fragments|.text.hci_send_acl_packet_buffer=.time_critical.hci_send_acl_packet_buffer"
        "Phase B.3: hci.c TX pump + ACL out -> RAM")
relocate_to_ram(lib/btstack/src/l2cap.c.o
        ".text.l2cap_run=.time_critical.l2cap_run|.text.l2cap_send=.time_critical.l2cap_send|.text.l2cap_send_prepared=.time_critical.l2cap_send_prepared|.text.l2cap_notify_channel_can_send=.time_critical.l2cap_notify_channel_can_send"
        "Phase B.3: l2cap.c TX pump + sends -> RAM")
relocate_to_ram(lib/cyw43-driver/src/cyw43_ll.c.o
        ".text.cyw43_ll_write_backplane_mem=.time_critical.cyw43_ll_write_backplane_mem|.text.cyw43_ll_read_backplane_mem=.time_critical.cyw43_ll_read_backplane_mem|.text.cyw43_write_backplane=.time_critical.cyw43_write_backplane|.text.cyw43_read_backplane.constprop.0=.time_critical.cyw43_read_backplane"
        "Phase B.3: cyw43 backplane accessors -> RAM")
relocate_to_ram(lib/tinyusb/src/device/usbd.c.o
        ".text.tud_task_ext=.time_critical.tud_task_ext|.text.usbd_edpt_xfer=.time_critical.usbd_edpt_xfer|.text.usbd_edpt_xfer_fifo=.time_critical.usbd_edpt_xfer_fifo"
        "Phase B.3: usbd.c task pump + xfer -> RAM")
relocate_to_ram(lib/tinyusb/src/portable/raspberrypi/rp2040/dcd_rp2040.c.o
        ".text.dcd_edpt_xfer=.time_critical.dcd_edpt_xfer|.text.dcd_edpt_iso_activate=.time_critical.dcd_edpt_iso_activate"
        "Phase B.3: dcd_rp2040.c submit + iso -> RAM")
relocate_to_ram(lib/tinyusb/src/class/audio/audio_device.c.o
        ".text.tud_audio_n_read=.time_critical.tud_audio_n_read|.text.tud_audio_n_write=.time_critical.tud_audio_n_write|.text.audiod_calc_tx_packet_sz.isra.0=.time_critical.audiod_calc_tx_packet_sz|.text.audiod_xfer_isr=.time_critical.audiod_xfer_isr"
        "Phase B.3: audio_device.c data path -> RAM")

# --- Phase B.4: input-report path tail (controller -> host) ---
# queue_try_* also back the audio inter-core FIFOs, so this helps both paths.
relocate_to_ram(lib/tinyusb/src/class/hid/hid_device.c.o
        ".text.tud_hid_n_report=.time_critical.tud_hid_n_report|.text.tud_hid_n_ready=.time_critical.tud_hid_n_ready|.text.hidd_xfer_cb=.time_critical.hidd_xfer_cb"
        "Phase B.4: hid_device.c report path -> RAM")
relocate_to_ram(src/common/pico_util/queue.c.o
        ".text.queue_try_add=.time_critical.queue_try_add|.text.queue_try_remove=.time_critical.queue_try_remove"
        "Phase B.4: pico_util queue add/remove -> RAM")
relocate_to_ram(src/rp2_common/pico_cyw43_arch/cyw43_arch.c.o
        ".text.cyw43_arch_poll=.time_critical.cyw43_arch_poll"
        "Phase B.4: cyw43_arch_poll -> RAM")
relocate_to_ram(lib/btstack/src/btstack_run_loop.c.o
        ".text.btstack_run_loop_base_poll_data_sources=.time_critical.btstack_run_loop_base_poll_data_sources|.text.btstack_run_loop_poll_data_sources_from_irq=.time_critical.btstack_run_loop_poll_data_sources_from_irq"
        "Phase B.4: btstack run-loop poll -> RAM")

# Phase B.5: RAM-resident mem ops. core1's audio loop calls memcpy/memset every
# frame (directly and inside libopus), and on RP2350 the Pico SDK uses newlib's
# memcpy/memset, which live in flash -- so core1 still executes flash code during
# steady-state audio. The pico/ROM mem-ops variant does not link on RP2350 (the
# bootrom no longer exports ROM_FUNC_MEMCPY/MEMSET), so src/ram_mem.c provides
# RAM-resident (.time_critical) overrides instead. Built with -fno-builtin and
# -fno-tree-loop-distribute-patterns so the compiler does not turn the copy loops
# back into calls to memcpy/memset (which would self-recurse). This completes the
# move of core1's audio loop into RAM (no per-frame XIP fetches left on core1).
# NOTE: re-validate audio at 150 MHz, since these replace newlib's mem ops.
#set_source_files_properties(src/ram_mem.c PROPERTIES COMPILE_OPTIONS
#        "-fno-builtin;-fno-tree-loop-distribute-patterns")

target_compile_options(ds5-bridge PRIVATE -fsigned-char -Wall -O3 -Wdouble-promotion -mfloat-abi=softfp)
target_compile_options(opus PRIVATE -O3 -mfloat-abi=softfp)

target_compile_definitions(ds5-bridge PRIVATE
        CYW43_LWIP=0
        PICO_FLASH_ASSUME_CORE1_SAFE=0
        CYW43_PIO_CLOCK_DIV_INT=2
)

pico_set_program_name(ds5-bridge "ds5-bridge")

pico_enable_stdio_uart(ds5-bridge 1)
pico_enable_stdio_usb(ds5-bridge 0)

target_link_libraries(ds5-bridge
        pico_stdlib
        pico_mem_ops
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
