//
// Created by awalol on 2026/3/4.
//

#include <bsp/board_api.h>
#include <tusb.h>

#include "bluetooth_packet.h"
#include "bt.h"
#include "config.h"
#include "log.h"
#include "state.h"

#define UAC1_ENTITY_SPK_FEATURE_UNIT 0x02
#define UAC1_ENTITY_MIC_FEATURE_UNIT 0x05

//--------------------------------------------------------------------+
// Audio Callback Functions
//--------------------------------------------------------------------+

//--------------------------------------------------------------------+
// UAC1 Helper Functions
//--------------------------------------------------------------------+

static bool __not_in_flash_func(audio10SetReqEntity)(tusb_control_request_t const *p_request, const uint8_t *pBuff) {
    // uint8_t channelNum = TU_U16_LOW(p_request->wValue);
    uint8_t ctrlSel = TU_U16_HIGH(p_request->wValue);
    uint8_t entityID = TU_U16_HIGH(p_request->wIndex);
    uint8_t *mute = nullptr;
    int16_t *volume = nullptr;
    if (entityID == UAC1_ENTITY_SPK_FEATURE_UNIT) {
        mute = &config.mute.speaker;
        volume = &config.volume.speaker;
    } else {
        mute = &config.mute.microphone;
        volume = &config.volume.microphone;
    }

    // If request is for our speaker feature unit
    if (entityID == UAC1_ENTITY_SPK_FEATURE_UNIT || entityID == UAC1_ENTITY_MIC_FEATURE_UNIT) {
        switch (ctrlSel) {
            case AUDIO10_FU_CTRL_MUTE:
                switch (p_request->bRequest) {
                    case AUDIO10_CS_REQ_SET_CUR:
                        // Only 1st form is supported
                        TU_VERIFY(p_request->wLength == 1);
                        if (*mute != pBuff[0]) {
                            *mute = pBuff[0];
                            if (entityID == UAC1_ENTITY_SPK_FEATURE_UNIT) {
                                updateVolume();
                            } else if (entityID == UAC1_ENTITY_MIC_FEATURE_UNIT) {
                                updateMicVolume();
                            }
                        }
                        TU_LOG2("    Set Mute: %d of entity: %u\r\n", *mute, entityID);
                        return true;

                    default:
                        return false;  // not supported
                }

            case AUDIO10_FU_CTRL_VOLUME:
                switch (p_request->bRequest) {
                    case AUDIO10_CS_REQ_SET_CUR:
                        // Only 1st form is supported
                        TU_VERIFY(p_request->wLength == 2);
                        int16_t newVolume = *((int16_t const *)pBuff);
                        if (*volume != newVolume) {
                            *volume = newVolume;
                            if (entityID == UAC1_ENTITY_SPK_FEATURE_UNIT) {
                                updateVolume();
                            } else if (entityID == UAC1_ENTITY_MIC_FEATURE_UNIT) {
                                updateMicVolume();
                            }
                        }
                        TU_LOG2("    Set Volume: %d dB of entity: %u\r\n", *volume, entityID);
                        return true;

                    default:
                        return false;  // not supported
                }

            // Unknown/Unsupported control
            default:
                TU_BREAKPOINT();
                return false;
        }
    }

    return false;
}

static bool  __not_in_flash_func(audio10GetReqEntity)(uint8_t rhport, tusb_control_request_t const *p_request) {
    // uint8_t channelNum = TU_U16_LOW(p_request->wValue);
    uint8_t ctrlSel = TU_U16_HIGH(p_request->wValue);
    uint8_t entityID = TU_U16_HIGH(p_request->wIndex);
    uint8_t *mute = nullptr;
    int16_t *volume = nullptr;
    if (entityID == UAC1_ENTITY_SPK_FEATURE_UNIT) {
        mute = &config.mute.speaker;
        volume = &config.volume.speaker;
    } else {
        mute = &config.mute.microphone;
        volume = &config.volume.microphone;
    }

    // If request is for our speaker feature unit
    if (entityID == UAC1_ENTITY_SPK_FEATURE_UNIT || entityID == UAC1_ENTITY_MIC_FEATURE_UNIT) {
        switch (ctrlSel) {
            case AUDIO10_FU_CTRL_MUTE:
                // Audio control mute cur parameter block consists of only one byte - we thus can send it right away
                // There does not exist a range parameter block for mute
                TU_LOG2("    Get Mute of entity: %u\r\n", entityID);
                return tud_audio_buffer_and_schedule_control_xfer(rhport, p_request, mute, 1);

            case AUDIO10_FU_CTRL_VOLUME:
                switch (p_request->bRequest) {
                    case AUDIO10_CS_REQ_GET_CUR:
                        TU_LOG2("    Get Volume of entity: %u\r\n", entityID);
                        {
                            return tud_audio_buffer_and_schedule_control_xfer(rhport, p_request, volume, sizeof(int16_t));
                        }

                    case AUDIO10_CS_REQ_GET_MIN:
                        TU_LOG2("    Get Volume min of entity: %u\r\n", entityID);
                        {
                            uint8_t min[2];
                            if (entityID == UAC1_ENTITY_SPK_FEATURE_UNIT) {
                                min[0] = 0x00;
                                min[1] = 0x9c;
                            } else {
                                min[0] = 0x00;
                                min[1] = 0x00;
                            }
                            return tud_audio_buffer_and_schedule_control_xfer(rhport, p_request, &min, sizeof(min));
                        }

                    case AUDIO10_CS_REQ_GET_MAX:
                        TU_LOG2("    Get Volume max of entity: %u\r\n", entityID);
                        {
                            uint8_t max[2];
                            if (entityID == UAC1_ENTITY_SPK_FEATURE_UNIT) {
                                max[0] = 0x00;
                                max[1] = 0x00;
                            } else {
                                max[0] = 0x00;
                                max[1] = 0x30;
                            }
                            return tud_audio_buffer_and_schedule_control_xfer(rhport, p_request, &max, sizeof(max));
                        }

                    case AUDIO10_CS_REQ_GET_RES:
                        TU_LOG2("    Get Volume res of entity: %u\r\n", entityID);
                        {
                            uint8_t res[2];
                            if (entityID == UAC1_ENTITY_SPK_FEATURE_UNIT) {
                                res[0] = 0x00;
                                res[1] = 0x01;
                            } else {
                                res[0] = 0x7a;
                                res[1] = 0x00;
                            }
                            return tud_audio_buffer_and_schedule_control_xfer(rhport, p_request, &res, sizeof(res));
                        }
                    // Unknown/Unsupported control
                    default:
                        TU_BREAKPOINT();
                        return false;
                }
                break;

            // Unknown/Unsupported control
            default:
                TU_BREAKPOINT();
                return false;
        }
    }

    return false;
}

// Invoked when audio class specific get request received for an entity
bool  __not_in_flash_func(tud_audio_get_req_entity_cb)(uint8_t rhport, tusb_control_request_t const *p_request) { return audio10GetReqEntity(rhport, p_request); }

// Invoked when audio class specific set request received for an entity
bool  __not_in_flash_func(tud_audio_set_req_entity_cb)(uint8_t rhport, tusb_control_request_t const *p_request, uint8_t *pBuff) { return audio10SetReqEntity(p_request, pBuff); }

void tud_hid_report_complete_cb(uint8_t instance, uint8_t const *report, uint16_t len) {}

void tud_suspend_cb(bool remote_wakeup_en) {
    LOGI("[USB PM] invoke tud_suspend_cb");
    btPowerOffController();
}

void __not_in_flash_func(usbInterruptLoop)() {
    if (tud_hid_ready()) {
        if (!tud_hid_report(0x01, getStatePacket()->data, ds5StatePacketSize)) {
            LOGE("[USBHID] tud_hid_report error");
        }
    }
}

// Invoked when received GET_REPORT control request
// Application must fill buffer report's content and return its length.
// Return zero will cause the stack to STALL request
uint16_t __not_in_flash_func(tud_hid_get_report_cb)(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t *buffer, uint16_t reqlen) {
    return getFeatureData(report_id, buffer, reqlen);
}

bool __not_in_flash_func(tud_audio_set_itf_cb)(uint8_t rhport, tusb_control_request_t const *p_request) {
    uint8_t const itf = tu_u16_low(p_request->wIndex);  // wInterface
    uint8_t const alt = tu_u16_low(p_request->wValue);  // bAlternateSetting
    bool active = (alt != 0);

    if (itf == 1) {
        config.audioActive = active;
        LOGI("[AUDIO] Set interface Speaker to alternate setting %d", alt);
    } else if (itf == 2) {  // ITF_NUM_AUDIO_STREAMING_IN (microphone)
        LOGI("[AUDIO] Set interface Microphone to alternate setting %d", alt);
        if (config.micActive != active) {
            config.micActive = active;
            needSendAudioSetup();
        }
    }

    return true;
}

// Invoked when received SET_REPORT control request or
// received data on OUT endpoint ( Report ID = 0, Type = 0 )
void __not_in_flash_func(tud_hid_set_report_cb)(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t const *buffer, uint16_t bufsize) {
    // INTERRUPT OUT
    if (report_id == 0) {
        if (bufsize < 1) {
            return;
        }
        switch (buffer[0]) {
            case 0x02: {
                const int size = MIN(bufsize - 1, subPacketControlSize);
                if (size < ds5ControlPayloadSize) {
                    LOGE("Received control sub packet with size %d, expected %d", bufsize - 1, ds5ControlPayloadSize);
                    break;
                }

                setControlPacket(buffer + 1, size);
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
    }
}
