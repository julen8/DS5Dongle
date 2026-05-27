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

#include "bluetoothPacket.h"
#include "bt.h"
#include "config.h"
#include "log.h"
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

constexpr auto inputChannels = 4;        // 固定不能修改
constexpr auto hapticChannels = 2;       // 固定不能修改
constexpr auto audioChannels = 2;        // 固定不能修改
constexpr auto audioRawElementSize = 4;  // 音频原始数据缓冲buf数量

constexpr auto readRawFrames = 48;          // 每次读取多少帧原始数据
constexpr auto rawSamplingRate = 48000;     // 固定不能修改
constexpr auto hapticOutSampleRate = 3000;  // 震动重采样输出的采样率

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
static_assert(sizeof(muteOpusPackage) == subPacketAudioSize, "sizeof(muteOpusPackage) != subPacketAudioSize");

struct AudioRawElement {
    bool inuse = false;
    WDL_ResampleSample data[audioResamplerInputFrames * audioChannels]{};
};

static struct {
    alignas(8) uint32_t audio_core1_stack[8192];
    WDL_Resampler hapticResampler{};
    WDL_Resampler audioResampler{};
    OpusEncoder* encoder = nullptr;
    queue_t audioRawFifo;
    AudioRawElement audioRawElementArray[audioRawElementSize]{};
    int8_t* hapticBuf = nullptr;
    AudioRawElement* currentAudioRawElement = nullptr;
    int hapticBufPos = 0;
    int audioBufPos = 0;
    bool needSendEndMuteOpusPackage = false;
    bool needSendFirstMuteOpusPackage = true;
    bool audioWaitData = true;
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

inline void freeAudioRawElement(AudioRawElement* audioRawElement) { audioRawElement->inuse = false; }

inline float getAudioGain() {
    static float lastSpeakerVolume = config.speakerVolume;
    static float audioGain = powf(10.0F, config.speakerVolume / 20.0F);

    if (lastSpeakerVolume != config.speakerVolume) {
        lastSpeakerVolume = config.speakerVolume;
        audioGain = powf(10.0F, config.speakerVolume / 20.0F);
    }

    return (config.mute[0] == 0) ? audioGain : 0.0F;
}

inline void processingRemainingData() {
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
        }

        return;
    }

    uint8_t* hapticBuf = nullptr;
    if (audio.hapticBufPos != 0) {
        memset(audio.hapticBuf + audio.hapticBufPos, 0, subPacketHapticSize - audio.hapticBufPos);
        audio.hapticBufPos = 0;
        hapticBuf = reinterpret_cast<uint8_t*>(audio.hapticBuf);
        audio.hapticBuf = nullptr;
    }

    if (hapticBuf != nullptr) {
        writeSubPacket(hapticBuf, subPacketType::haptic);
        audio.needSendEndMuteOpusPackage = true;
        return;
    }

    // 补发一个静音包，避免下次开始播放时出现一个杂音
    if (audio.audioWaitData && audio.needSendEndMuteOpusPackage) {
        audio.needSendEndMuteOpusPackage = false;

        if (auto* audioBuff = getBufferForSubPacket(subPacketType::audio); audioBuff != nullptr) {
            memcpy(audioBuff, muteOpusPackage, subPacketAudioSize);
            writeSubPacket(audioBuff, subPacketType::audio);
        } else {
            LOGW("getBufferForSubPacket audio failed");
        }

        if (auto* buffer = getBufferForSubPacket(subPacketType::haptic); buffer != nullptr) {
            memset(buffer, 0, subPacketHapticSize);
            writeSubPacket(buffer, subPacketType::haptic);
        } else {
            LOGW("getBufferForSubPacket haptic failed");
        }
    }
}

void audioLoop() {
    if (tud_audio_available() == 0) {
        if (!config.audioActive) {
            audio.needSendFirstMuteOpusPackage = true;
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
    if (hapticResampleInBuf == nullptr) {
        LOGE("hapticResampleInBuf is null");
        return;
    }

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
            } else if (audio.audioBufPos == audioResamplerInputFrames * audioChannels / 2) {
                // 音频在另外一个core通知发送会有问题，所以在这里来通知，假设到一半的时候opus已经编码好了
                btRequestSend();
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
        if (audio.hapticBuf == nullptr) {
            audio.hapticBuf = reinterpret_cast<int8_t*>(getBufferForSubPacket(subPacketType::haptic));
            if (audio.hapticBuf == nullptr) {
                LOGE("getBufferForSubPacket for haptic failed");
                break;
            }
        }

        int valLeftChannel = static_cast<int>(hapticResampleOutBuf[i * 2] * (INT8_MAX + 1));
        int valRightChannel = static_cast<int>(hapticResampleOutBuf[i * 2 + 1] * (INT8_MAX + 1));
        audio.hapticBuf[audio.hapticBufPos++] = static_cast<int8_t>(std::clamp(valLeftChannel, INT8_MIN, INT8_MAX));
        audio.hapticBuf[audio.hapticBufPos++] = static_cast<int8_t>(std::clamp(valRightChannel, INT8_MIN, INT8_MAX));

        if (audio.hapticBufPos == subPacketHapticSize) {
            // 如果是第一次发送一个静音audio包，避免haptic包等待
            if (audio.needSendFirstMuteOpusPackage) {
                audio.needSendFirstMuteOpusPackage = false;
                if (auto* audioBuff = getBufferForSubPacket(subPacketType::audio); audioBuff != nullptr) {
                    memcpy(audioBuff, muteOpusPackage, subPacketAudioSize);
                    writeSubPacket(audioBuff, subPacketType::audio);
                } else {
                    LOGW("getBufferForSubPacket failed");
                }
            }

            writeSubPacket(reinterpret_cast<uint8_t*>(audio.hapticBuf), subPacketType::haptic);
            audio.hapticBuf = nullptr;
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
        audio.audioWaitData = true;
        queue_remove_blocking(&audio.audioRawFifo, static_cast<void*>(&audioRawElement));
        audio.audioWaitData = false;
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

        auto* audioOut = getBufferForSubPacket(subPacketType::audio);
        if (audioOut == nullptr) {
            LOGW("Opus get out buf err");
            continue;
        }
        if (const auto ret = opus_encode_float(audio.encoder, resampleOutBuf, audioResamplerOutputFrames, audioOut, subPacketAudioSize); ret < 0) {
            LOGE("opus_encode_float failed:%d", ret);
            freeSubPacket(audioOut, subPacketType::audio);
            continue;
        }
        writeSubPacket(audioOut, subPacketType::audio);
    }
}

void audioInit() {
    audio.hapticResampler.SetMode(true, 0, false);
    audio.hapticResampler.SetRates(rawSamplingRate, hapticOutSampleRate);
    audio.hapticResampler.SetFeedMode(true);
    audio.hapticResampler.Prealloc(hapticChannels, readRawFrames, readRawFrames / (rawSamplingRate / hapticOutSampleRate));  // 每次输入 48 帧，输出 3 帧（48/16）
    queue_init(&audio.audioRawFifo, sizeof(AudioRawElement*), audioRawElementSize);
    multicore_launch_core1_with_stack(core1Entry, audio.audio_core1_stack, sizeof(audio.audio_core1_stack));
}
