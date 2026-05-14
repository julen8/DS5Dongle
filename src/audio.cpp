//
// Created by awalol on 2026/3/5.
//

#include "audio.h"

#include <opus.h>
#include <pico/multicore.h>
#include <pico/util/queue.h>
#include <resample.h>
#include <tusb.h>

#include <algorithm>
#include <cmath>

#include "bt.h"
#include "config.h"
#include "log.h"
#include "usb.h"
#include "utils.h"

/*
 *最终发送的 震动buf 和 音频buf 的大小分别 -> 64 , 200
 *震动是经过48000hz重采样到3000hz
 *音频是需要经过opus编码
 *所以每一个发送的蓝牙包:
 *  震动数据是64字节 3khz 8位 对应48khz 16位 -> 64 * (48 / 3) * 2 = 1024 *2 = 2048字节 对应 512 frames
 *  因为音频和震动原始数据都是48khz且都是2声道，所以原始数据一样多
 *  所以对于音频来说需要把2048字节(512 frames)的数据编码成200字节的opus
 * 处理方案:
 * 震动: 512 frames(2声道 48khz 16bit) ->         重采样 ->                                                 32 frames(2声道 3khz 8bit) | -> 打包发送
 * 音频: 512 frames(2声道 48khz 16bit) -> 重采样 -> 480 frames(2声道 48khz WDL_ResampleSample) -> opus编码 -> 200 字节编码后数据          | -> 打包发送
 *                         音频重采样:将512frames变成480frames数据，因为opus不能处理512frames为了对齐
 */

constexpr auto inputChannels = 4;         // 固定不能修改
constexpr auto hapticChannels = 2;        // 固定不能修改
constexpr auto audioChannels = 2;         // 固定不能修改
constexpr auto audioRawElementSize = 3;   // 音频原始数据缓冲buf数量
constexpr auto audioOpusElementSize = 3;  // 音频opus编码后缓冲buf数量

constexpr auto readRawFrames = 48;          // 每次读取多少帧原始数据
constexpr auto rawSamplingRate = 48000;     // 固定不能修改
constexpr auto hapticOutSampleRate = 3000;  // 震动重采样输出的采样率

constexpr auto hapticDataSizeInBtPacket = 64;  // 蓝牙包中震动数据的大小
constexpr auto audioDataSizeInBtPacket = 200;  // 蓝牙包中音频数据的大小

constexpr auto audioResamplerInputFrames = 512;
constexpr auto audioResamplerOutputFrames = 480;

// 一个静音的opus编码后的包，通过传入静音的pcm编码后得到
constexpr uint8_t muteOpusPackage[] = {
    0xF4, 0xFF, 0xFE, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00,
    00,   00,   00,   00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00,
    00,   00,   00,   00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00,
    00,   00,   00,   00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00,
    00,   00,   00,   00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00,
};
static_assert(sizeof(muteOpusPackage) == audioDataSizeInBtPacket, "sizeof(muteOpusPackage) != audioDataSizeInBtPacket");

struct AudioRawElement {
    bool inuse = false;
    WDL_ResampleSample data[audioResamplerInputFrames * audioChannels]{};
};

struct AudioOpusElement {
    bool inuse = false;
    uint8_t data[audioDataSizeInBtPacket]{};
};

static struct {
    alignas(8) uint32_t audio_core1_stack[8192];
    WDL_Resampler hapticResampler{};
    WDL_Resampler audioResampler{};
    OpusEncoder* encoder = nullptr;
    bool plugHeadset = false;
    uint8_t reportSeqCounter = 0;
    uint8_t packetCounter = 0;
    queue_t audioRawFifo;
    queue_t audioOpusFifo;
    AudioRawElement audioRawElementArray[audioRawElementSize]{};
    AudioOpusElement audioOpusElementArray[audioOpusElementSize]{};
    int8_t hapticBuf[hapticDataSizeInBtPacket]{};
    AudioRawElement* currentAudioRawElement = nullptr;
    int hapticBufPos = 0;
    int audioBufPos = 0;
    bool needSendEndMuteOpusPackage = false;
} audio{};

inline AudioRawElement* getAudioRawElement() {
    for (auto& elem : audio.audioRawElementArray) {
        if (!elem.inuse) {
            elem.inuse = true;
            return &elem;
        }
    }

    return nullptr;
}

inline AudioOpusElement* getAudioOpusElement() {
    for (auto& elem : audio.audioOpusElementArray) {
        if (!elem.inuse) {
            elem.inuse = true;
            return &elem;
        }
    }

    return nullptr;
}

inline void freeAudioRawElement(AudioRawElement* audioRawElement) { audioRawElement->inuse = false; }
inline void freeAudioOpusElement(AudioOpusElement* audioOpusElement) { audioOpusElement->inuse = false; }

void setHeadset(const bool state) { audio.plugHeadset = state; }

inline float getAudioGain() {
    static float lastSpeakerVolume = config.speakerVolume;
    static float audioGain = powf(10.0F, config.speakerVolume / 20.0F);

    if (lastSpeakerVolume != config.speakerVolume) {
        lastSpeakerVolume = config.speakerVolume;
        audioGain = powf(10.0F, config.speakerVolume / 20.0F);
    }

    return (mute[0] == 0) ? audioGain : 0.0F;
}

inline void packAndWriteBtDataPacket(const int8_t* hapticBuf, const uint8_t* audioBuf) {
    constexpr auto reportSize = 398;
    constexpr uint8_t reportId = 0x36;
    uint8_t pkt[reportSize]{};
    pkt[0] = reportId;
    pkt[1] = audio.reportSeqCounter << 4;
    audio.reportSeqCounter = (audio.reportSeqCounter + 1) & 0x0F;
    pkt[2] = 0x11 | 0 << 6 | 1 << 7;
    pkt[3] = 7;
    pkt[4] = 0b11111110;
    const auto bufLen = config.audioBufferLength;
    pkt[5] = bufLen;
    pkt[6] = bufLen;
    pkt[7] = bufLen;
    pkt[8] = bufLen;  // 这 4 个字节的作用未知，调整没有效果
    pkt[9] = bufLen;  // audio buffer length 只有调整这个字节生效。
    pkt[10] = audio.packetCounter++;
    pkt[11] = 0x12 | 0 << 6 | 1 << 7;
    pkt[12] = hapticDataSizeInBtPacket;
    if (hapticBuf != nullptr) {
        memcpy(pkt + 13, hapticBuf, hapticDataSizeInBtPacket);
    } else {
        memset(pkt + 13, 0, hapticDataSizeInBtPacket);
    }
    pkt[77] = (audio.plugHeadset ? 0x16 : 0x13) | 0 << 6 | 1 << 7;  // Speaker: 0x13
                                                                    // L Headset Mono: 0x14
                                                                    // L Headset R Speaker: 0x15
                                                                    // Headset: 0x16
    pkt[78] = audioDataSizeInBtPacket;
    if (audioBuf != nullptr) {
        memcpy(pkt + 79, audioBuf, audioDataSizeInBtPacket);
    } else {
        memcpy(pkt + 79, muteOpusPackage, audioDataSizeInBtPacket);
    }

    bt_write(pkt, sizeof(pkt), true);
}

inline void packAndWriteBtDataPacket(const int8_t* hapticBuf) {
    if (AudioOpusElement* audioOpusElement = nullptr; !queue_try_remove(&audio.audioOpusFifo, static_cast<void*>(&audioOpusElement))) {
        LOGW("audioOpusFifo remove failed, send muteOpusPackage");
        packAndWriteBtDataPacket(hapticBuf, nullptr);
    } else {
        packAndWriteBtDataPacket(hapticBuf, audioOpusElement->data);
        freeAudioOpusElement(audioOpusElement);
    }
}

inline void processingRemainingData() {
    const int8_t* hapticBuf = nullptr;

    if (audio.audioBufPos != 0) {
        if (audio.currentAudioRawElement == nullptr) {
            LOGW("audioBufPos=%d but currentAudioRawElement is null, discarding", audio.audioBufPos);
            audio.audioBufPos = 0;
        } else {
            memset(audio.currentAudioRawElement->data + audio.audioBufPos, 0, (std::size(audio.currentAudioRawElement->data) - audio.audioBufPos) * sizeof(audio.currentAudioRawElement->data[0]));

            if (!queue_try_add(&audio.audioRawFifo, static_cast<void*>(&audio.currentAudioRawElement))) {
                LOGW("audio_fifo add failed");
                freeAudioRawElement(audio.currentAudioRawElement);
            }

            audio.currentAudioRawElement = nullptr;
            audio.audioBufPos = 0;
        }  // end else (currentAudioRawElement != nullptr)
    }

    if (audio.hapticBufPos != 0) {
        memset(audio.hapticBuf + audio.hapticBufPos, 0, (std::size(audio.hapticBuf) - audio.hapticBufPos) * sizeof(audio.hapticBuf[0]));
        audio.hapticBufPos = 0;
        hapticBuf = audio.hapticBuf;
    }

    if (hapticBuf != nullptr) {
        packAndWriteBtDataPacket(hapticBuf);
        audio.needSendEndMuteOpusPackage = true;
        return;
    }

    if (!queue_is_empty(&audio.audioOpusFifo)) {
        packAndWriteBtDataPacket(nullptr);
        audio.needSendEndMuteOpusPackage = true;
        return;
    }

    // 补发一个静音包，避免下次开始播放时出现一个杂音
    if (audio.needSendEndMuteOpusPackage) {
        audio.needSendEndMuteOpusPackage = false;
        packAndWriteBtDataPacket(nullptr);
    }
}

void audioLoop() {
    if (tud_audio_available() == 0) {
        if (!config.audioActive) {
            // usb已经停止发送pcm数据了,但是这里需要把剩下的缓存的数据处理完
            processingRemainingData();
        }

        return;
    }

    static int16_t raw[readRawFrames * inputChannels];
    const auto bytesRead = tud_audio_read(raw, sizeof(raw));
    const auto actualRawFrames = bytesRead / (inputChannels * sizeof(int16_t));
    if (actualRawFrames == 0) {
        return;
    }

    audio.needSendEndMuteOpusPackage = true;

    WDL_ResampleSample* hapticResampleInBuf = nullptr;
    const int nFrames = audio.hapticResampler.ResamplePrepare(static_cast<int>(actualRawFrames), hapticChannels, &hapticResampleInBuf);

    const float audioGain = getAudioGain();
    for (int i = 0; i < nFrames; i++) {
        if (audio.currentAudioRawElement == nullptr) {
            audio.currentAudioRawElement = getAudioRawElement();
            if (audio.currentAudioRawElement == nullptr) {
                LOGE("no free AudioRawElement");
            }
        }

        if (audio.currentAudioRawElement != nullptr) {
            audio.currentAudioRawElement->data[audio.audioBufPos++] = static_cast<WDL_ResampleSample>(raw[i * inputChannels]) / (INT16_MAX + 1) * audioGain;
            audio.currentAudioRawElement->data[audio.audioBufPos++] = static_cast<WDL_ResampleSample>(raw[i * inputChannels + 1]) / (INT16_MAX + 1) * audioGain;
            if (audio.audioBufPos == audioResamplerInputFrames * audioChannels) {
                audio.audioBufPos = 0;

                if (!queue_try_add(&audio.audioRawFifo, static_cast<void*>(&audio.currentAudioRawElement))) {
                    LOGW("audio_fifo add failed");
                    freeAudioRawElement(audio.currentAudioRawElement);
                }

                audio.currentAudioRawElement = nullptr;
                // Note: next iteration will re-acquire a free element at the top of the loop.
            }
        }

        hapticResampleInBuf[i * 2] = static_cast<WDL_ResampleSample>(raw[i * inputChannels + 2]) / (INT16_MAX + 1);
        hapticResampleInBuf[i * 2 + 1] = static_cast<WDL_ResampleSample>(raw[i * inputChannels + 3]) / (INT16_MAX + 1);
    }

    // 3. 48kHz -> 3kHz 重采样
    // actual: nFrames / (rawSamplingRate / hapticOutSampleRate) * hapticOutputChannels
    // max : readRawFrames / (rawSamplingRate / hapticOutSampleRate) * hapticOutputChannels
    static WDL_ResampleSample hapticResampleOutBuf[readRawFrames / (rawSamplingRate / hapticOutSampleRate) * hapticChannels];
    const int hapticResampleOutFrames = audio.hapticResampler.ResampleOut(hapticResampleOutBuf, nFrames, nFrames / (rawSamplingRate / hapticOutSampleRate), hapticChannels);

    // 4. 转换为int8并缓冲，满64字节即组包发送
    for (int i = 0; i < hapticResampleOutFrames; i++) {
        int val_l = static_cast<int>(hapticResampleOutBuf[i * 2] * (INT8_MAX + 1));
        int val_r = static_cast<int>(hapticResampleOutBuf[i * 2 + 1] * (INT8_MAX + 1));
        audio.hapticBuf[audio.hapticBufPos++] = static_cast<int8_t>(std::clamp(val_l, INT8_MIN, INT8_MAX));
        audio.hapticBuf[audio.hapticBufPos++] = static_cast<int8_t>(std::clamp(val_r, INT8_MIN, INT8_MAX));

        if (audio.hapticBufPos == hapticDataSizeInBtPacket) {
            packAndWriteBtDataPacket(audio.hapticBuf);
            audio.hapticBufPos = 0;
        }
    }
}

void core1Entry() {
    int error = 0;
    audio.encoder = opus_encoder_create(48000, audioChannels, OPUS_APPLICATION_AUDIO, &error);
    if (error != 0) {
        LOGE("OpusEncoder create failed");
        return;
    }
    opus_encoder_ctl(audio.encoder, OPUS_SET_EXPERT_FRAME_DURATION(OPUS_FRAMESIZE_10_MS));
    opus_encoder_ctl(audio.encoder, OPUS_SET_BITRATE(200 * 8 * 100));
    opus_encoder_ctl(audio.encoder, OPUS_SET_VBR(false));
    opus_encoder_ctl(audio.encoder, OPUS_SET_COMPLEXITY(0));  // max 4
    audio.audioResampler.SetMode(true, 0, false);
    audio.audioResampler.SetRates(51200, 48000);
    audio.audioResampler.SetFeedMode(true);
    audio.audioResampler.Prealloc(audioChannels, audioResamplerInputFrames, audioResamplerOutputFrames);

    for (;;) {
        AudioRawElement* audioRawElement = nullptr;
        queue_remove_blocking(&audio.audioRawFifo, static_cast<void*>(&audioRawElement));
        // 将 audioResamplerInputFrames frames 重采样成 audioResamplerOutputFrames frames 以解决噪音问题。感谢 @Junhoo
        WDL_ResampleSample* inBuf = nullptr;
        const int frames = audio.audioResampler.ResamplePrepare(audioResamplerInputFrames, audioChannels, &inBuf);

        const int framesToCopy = std::min(frames, audioResamplerInputFrames);
        for (int i = 0; i < framesToCopy * audioChannels; i++) {
            inBuf[i] = audioRawElement->data[i];
        }
        freeAudioRawElement(audioRawElement);

        if (frames > framesToCopy) {
            LOGI("frames > framesToCopy: %d", frames - framesToCopy);
            memset(inBuf + (framesToCopy * audioChannels), 0, (frames - framesToCopy) * audioChannels * sizeof(WDL_ResampleSample));
        }

        static WDL_ResampleSample resampleOutBuf[audioResamplerOutputFrames * audioChannels];
        audio.audioResampler.ResampleOut(resampleOutBuf, frames, audioResamplerOutputFrames, audioChannels);

        auto* audioOpusElement = getAudioOpusElement();
        if (audioOpusElement == nullptr) {
            LOGW("Opus get out buf err");
            continue;
        }
        if (const auto ret = opus_encode_float(audio.encoder, resampleOutBuf, audioResamplerOutputFrames, audioOpusElement->data, audioDataSizeInBtPacket); ret < 0) {
            LOGE("opus_encode_float failed:%d", ret);
            freeAudioOpusElement(audioOpusElement);
            continue;
        }

        if (!queue_try_add(&audio.audioOpusFifo, static_cast<void*>(&audioOpusElement))) {
            LOGW("audioOpusFifo add failed");
            freeAudioOpusElement(audioOpusElement);
        }
    }
}

void audioInit() {
    audio.hapticResampler.SetMode(true, 0, false);
    audio.hapticResampler.SetRates(rawSamplingRate, hapticOutSampleRate);
    audio.hapticResampler.SetFeedMode(true);
    audio.hapticResampler.Prealloc(hapticChannels, readRawFrames, readRawFrames / (rawSamplingRate / hapticOutSampleRate));  // 每次输入 48 帧，输出 3 帧（48/16）
    queue_init(&audio.audioRawFifo, sizeof(AudioRawElement*), audioRawElementSize);
    queue_init(&audio.audioOpusFifo, sizeof(AudioOpusElement*), audioOpusElementSize);
    multicore_launch_core1_with_stack(core1Entry, audio.audio_core1_stack, sizeof(audio.audio_core1_stack));
}
