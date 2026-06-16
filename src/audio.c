//
// Created by awalol on 2026/3/5.
//

#include "audio.h"

#include <assert.h>
#include <opus.h>
#include <pico/multicore.h>
#include <pico/util/queue.h>
#include <stdatomic.h>
#include <stdint.h>
#include <tusb.h>

#include "bluetoothPacket.h"
#include "bt.h"
#include "config.h"
#include "lerp_resampler.h"
#include "log.h"
#include "pico/flash.h"

#define USE_CIC_FOR_HAPTIC 1

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

constexpr int inputChannels = 4;                    // 固定不能修改
[[maybe_unused]] constexpr int hapticChannels = 2;  // 固定不能修改
constexpr int audioChannels = 2;                    // 固定不能修改
constexpr int audioResamplerInputFrames = 32;       // 512 / audioResamplerOutToOpusInCount
constexpr int audioResamplerOutputFrames = 30;      // 480 / audioResamplerOutToOpusInCount
constexpr int audioRawElementSize = 16;             // 音频原始数据缓冲buf数量
constexpr int readRawFrames = 48;                   // 每次读取多少帧原始数据
constexpr int rawSamplingRate = 48000;              // 固定不能修改
constexpr int hapticOutSampleRate = 3000;           // 震动重采样输出的采样率
constexpr int audioResamplerOutToOpusInCount = 16;  // 需要把 audioResamplerOutputFrames 的数据累积到 audioResamplerOutToOpusInCount 个才进行一次 opus 编码
constexpr int audioOpusInFrames = 480;              // opus编码每次输入的帧数，固定为480，不能修改
constexpr int micChannels = 1;                      // 从ds5收到的麦克风数据的声道数量
constexpr int micFrames = 480;                      // 一个mic包包含的数据帧数
constexpr int micOpusSize = 71;                     // bytes per opus-encoded mic frame from the DualSense
constexpr int micPcmElementSize = 2;
constexpr int micOpusElementSize = 2;

struct AudioRawElement {
    int16_t data[audioResamplerInputFrames * audioChannels];
    atomic_bool inuse;
};

struct MicOpuselement {
    uint8_t data[micOpusSize];
    atomic_bool inuse;
};

struct MicPcmElement {
    int16_t data[micFrames * micChannels];
    int frames;
    atomic_bool inuse;
};

static struct {
    alignas(8) uint32_t audio_core1_stack[8192];
    OpusEncoder* encoder;
    OpusDecoder* decoder;
    queue_t audioPcmFifo;
    queue_t micOpusFifo;
    queue_t micPcmFifo;
    struct AudioRawElement audioRawElementArray[audioRawElementSize];
    struct MicOpuselement micOpusElementArray[micOpusElementSize];
    struct MicPcmElement micPcmElementArray[micPcmElementSize];
    int8_t* hapticBuf;
    struct AudioRawElement* currentAudioRawElement;
    lerpRs audioResampler;
    uint32_t rawPos;
    int hapticBufPos;
    int audioBufPos;
    bool needClean;
    bool needSendFirstMuteOpusPackage;
    // audioWaitData 由 core1 写、core0 读，须加 volatile 确保跨核可见
    volatile bool audioWaitData;
} audio = {};

#if USE_CIC_FOR_HAPTIC
// 3 阶 CIC，立体声，抽取因子 R=16，差分延迟 M=1
struct HapticCicState {
    // 3 级积分器（工作在高速率 48kHz）
    int32_t i1L, i2L, i3L;
    int32_t i1R, i2R, i3R;
    // 3 级梳状延迟线（工作在低速率 3kHz）
    int32_t c1L_z, c2L_z, c3L_z;
    int32_t c1R_z, c2R_z, c3R_z;
};

inline void resetHapticCicState(struct HapticCicState* state) {
    state->i1L = state->i2L = state->i3L = 0;
    state->i1R = state->i2R = state->i3R = 0;
    state->c1L_z = state->c2L_z = state->c3L_z = 0;
    state->c1R_z = state->c2R_z = state->c3R_z = 0;
}

static struct HapticCicState hapticCic;
#endif

static inline void setMuteOpusPackage(uint8_t* opusPacket) {
    opusPacket[0] = 0xF4;  // TOC byte: 48kHz, 2 channels, 10ms frames, CBR, no VBR header
    opusPacket[1] = 0xFF;
    opusPacket[2] = 0xFE;
    memset(opusPacket + 3, 0, subPacketAudioSize - 3);
}

static inline struct AudioRawElement* __not_in_flash_func(getAudioRawElement)() {
    for (int i = 0; i < audioRawElementSize; ++i) {
        // 原子地占用空闲元素，避免与 core1 的释放产生跨核竞争
        bool expected = false;
        if (atomic_compare_exchange_strong(&audio.audioRawElementArray[i].inuse, &expected, true)) {
            return &audio.audioRawElementArray[i];
        }
    }

    return nullptr;
}

static inline void __not_in_flash_func(freeAudioRawElement)(struct AudioRawElement* audioRawElement) { atomic_store(&audioRawElement->inuse, false); }

static inline struct MicPcmElement* __not_in_flash_func(getMicPcmElement)() {
    for (int i = 0; i < micPcmElementSize; ++i) {
        // 原子地占用空闲元素，避免与 core1 的释放产生跨核竞争
        bool expected = false;
        if (atomic_compare_exchange_strong(&audio.micPcmElementArray[i].inuse, &expected, true)) {
            return &audio.micPcmElementArray[i];
        }
    }

    return nullptr;
}

static inline void __not_in_flash_func(freeMicPcmElement)(struct MicPcmElement* element) { atomic_store(&element->inuse, false); }

static inline struct MicOpuselement* __not_in_flash_func(getMicOpusElement)() {
    for (int i = 0; i < micOpusElementSize; ++i) {
        // 原子地占用空闲元素，避免与 core1 的释放产生跨核竞争
        bool expected = false;
        if (atomic_compare_exchange_strong(&audio.micOpusElementArray[i].inuse, &expected, true)) {
            return &audio.micOpusElementArray[i];
        }
    }

    return nullptr;
}

static inline void __not_in_flash_func(freeMicOpusElement)(struct MicOpuselement* element) { atomic_store(&element->inuse, false); }

static inline void cleanRemainingData() {
    {
        struct AudioRawElement* audioRawElement = audio.currentAudioRawElement;
        if (audioRawElement == nullptr) {
            audioRawElement = getAudioRawElement();
        }
        if (audioRawElement != nullptr) {
            if (!queue_try_add(&audio.audioPcmFifo, &audioRawElement)) {
                LOGW("audio_fifo add failed");
                freeAudioRawElement(audioRawElement);
            }
        }
    }

    {
        freeSubPacket((uint8_t*)(audio.hapticBuf), subPacketTypeHaptic);
        audio.rawPos = 0;
        audio.hapticBufPos = 0;
        audio.hapticBuf = nullptr;
#if USE_CIC_FOR_HAPTIC
        resetHapticCicState(&hapticCic);
#endif
        audio.audioBufPos = 0;
        audio.currentAudioRawElement = nullptr;
    }

    cleanAllCachedAudio();
    cleanAllCachedHaptic();
}

void __not_in_flash_func(audioLoop)() {
    // Mic playback: drain decoded mic PCM into the USB IN endpoint
    static struct MicPcmElement* pcmElement = nullptr;
    if (queue_try_remove(&audio.micPcmFifo, (void*)&pcmElement)) {
        // The controller mic is mono, but the USB descriptor presents a 2-channel
        // mic (matching the real DS5) so Windows doesn't conflict with its cached
        // DS5 audio format. Duplicate each mono sample into L and R.
        static int16_t micStereo[micFrames * 2];
        for (int i = 0; i < pcmElement->frames; i++) {
            micStereo[2 * i] = pcmElement->data[i];
            micStereo[(2 * i) + 1] = pcmElement->data[i];
        }
        freeMicPcmElement(pcmElement);
        const uint16_t stereoLen = (uint16_t)(pcmElement->frames * 2 * sizeof(int16_t));
        uint16_t written = tud_audio_write(micStereo, stereoLen);
        if (written != stereoLen) {
            // Gated behind ENABLE_VERBOSE: when the host has not opened the mic
            // interface (the common case -- most games never do) tud_audio_write
            // short-writes every frame, so an unconditional log would flood
            // core0's hot path with the newlib formatting chain.
            LOGE("[Audio] Warning: USB mic FIFO wrote %u/%u bytes", written, stereoLen);
        }
    }

    if (tud_audio_available() == 0) {
        if (!config.audioActive) {
            // usb已经停止发送pcm数据了,但是这里需要把剩下的缓存的数据处理完
            if (audio.needClean) {
                audio.needClean = false;
                audio.needSendFirstMuteOpusPackage = true;
                cleanRemainingData();
            }
        }

        return;
    }

    static int16_t raw[readRawFrames * inputChannels];
    const int bytesRead = tud_audio_read(raw + (audio.rawPos * inputChannels), sizeof(raw) - (audio.rawPos * inputChannels * sizeof(int16_t)));
    const int actualRawFrames = bytesRead / (inputChannels * sizeof(int16_t));
    if (bytesRead % (inputChannels * sizeof(int16_t)) != 0) {
        LOGW("bytesRead %% (inputChannels * sizeof(int16_t)):%d", bytesRead % (inputChannels * sizeof(int16_t)));
    }
    if (actualRawFrames == 0) {
        return;
    }
    audio.rawPos += actualRawFrames;
    if (audio.rawPos < readRawFrames) {
        return;
    }
    if (audio.rawPos != readRawFrames) {
        LOGW("audio.rawPos != readRawFrames: %lu", audio.rawPos);
    }

    audio.rawPos = 0;
    audio.needClean = true;

    // ── 1. 音频原始数据缓冲（送 core1 重采样 + Opus 编码）
    for (uint32_t i = 0; i < readRawFrames; i++) {
        if (audio.currentAudioRawElement == nullptr) {
            audio.currentAudioRawElement = getAudioRawElement();
            if (audio.currentAudioRawElement == nullptr) {
                LOGE("no free AudioRawElement");
            }
        }

        if (audio.currentAudioRawElement != nullptr) {
            audio.currentAudioRawElement->data[audio.audioBufPos++] = raw[i * inputChannels];
            audio.currentAudioRawElement->data[audio.audioBufPos++] = raw[i * inputChannels + 1];
            if (audio.audioBufPos == audioResamplerInputFrames * audioChannels) {
                audio.audioBufPos = 0;

                if (!queue_try_add(&audio.audioPcmFifo, &audio.currentAudioRawElement)) {
                    LOGW("audio_fifo add failed");
                    freeAudioRawElement(audio.currentAudioRawElement);
                }

                audio.currentAudioRawElement = nullptr;
            } else if (audio.audioBufPos == audioResamplerInputFrames * audioChannels / 2) {
                // 音频在另外一个core通知发送会有问题，所以在这里来通知，假设到一半的时候opus已经编码好了
                btRequestSend();
            }
        }
    }

    // ── 2. 震动：48kHz→3kHz（16:1）
    constexpr int hapticDecimFactor = rawSamplingRate / hapticOutSampleRate;  // = 16
    constexpr int hapticOutFrames = readRawFrames / hapticDecimFactor;

    for (int i = 0; i < hapticOutFrames; i++) {
#if USE_CIC_FOR_HAPTIC
        // 3 阶 CIC 抽取
        //  CIC 总增益 = R^N = 16^3 = 4096，对应 >>12
        //  int16 → int8 再 >>8，合计右移 20 位
        //  输出范围理论上 [-128, +128]，故需饱和到 int8
        const int base = i * hapticDecimFactor;

        // ── 高速率：16 次积分（纯加法，L/R 并行做提高 cache 命中率）──
        for (int j = 0; j < hapticDecimFactor; j++) {
            const int idx = (base + j) * inputChannels;
            const int32_t xL = raw[idx + 2];
            const int32_t xR = raw[idx + 3];

            hapticCic.i1L += xL;
            hapticCic.i2L += hapticCic.i1L;
            hapticCic.i3L += hapticCic.i2L;
            hapticCic.i1R += xR;
            hapticCic.i2R += hapticCic.i1R;
            hapticCic.i3R += hapticCic.i2R;
        }

        // ── 低速率：3 级梳状（纯减法）──
        const int32_t c1L = hapticCic.i3L - hapticCic.c1L_z;
        hapticCic.c1L_z = hapticCic.i3L;
        const int32_t c2L = c1L - hapticCic.c2L_z;
        hapticCic.c2L_z = c1L;
        const int32_t c3L = c2L - hapticCic.c3L_z;
        hapticCic.c3L_z = c2L;

        const int32_t c1R = hapticCic.i3R - hapticCic.c1R_z;
        hapticCic.c1R_z = hapticCic.i3R;
        const int32_t c2R = c1R - hapticCic.c2R_z;
        hapticCic.c2R_z = c1R;
        const int32_t c3R = c2R - hapticCic.c3R_z;
        hapticCic.c3R_z = c2R;

        // ── 增益补偿 + 位深转换 + 饱和 ──
        int32_t yL = c3L >> 20;
        int32_t yR = c3R >> 20;
        if (yL > 127) {
            yL = 127;
        } else if (yL < -128) {
            yL = -128;
        }
        if (yR > 127) {
            yR = 127;
        } else if (yR < -128) {
            yR = -128;
        }

        const int8_t outL = (int8_t)yL;
        const int8_t outR = (int8_t)yR;
#else
        // 整数 box-filter 抽取 48kHz→3kHz（16:1）
        // 每组 16 个 int16 样本累加 → 算术右移 12 位（>>4 平均 + >>8 缩放 int16→int8）
        // 最大值：16×32767=524272，>>12=127，恰好适配 int8
        int32_t sumL = 0;
        int32_t sumR = 0;
        const int base = i * hapticDecimFactor;
        for (int j = 0; j < hapticDecimFactor; j++) {
            sumL += raw[((base + j) * inputChannels) + 2];
            sumR += raw[((base + j) * inputChannels) + 3];
        }
        // 16×max_int16=524272, >>12=127; 16×min_int16=-524288, >>12=-128 → 值域恰好 [-128,127]
        const int8_t outL = (int8_t)(sumL >> 12);
        const int8_t outR = (int8_t)(sumR >> 12);
#endif

        if (audio.hapticBuf == nullptr) {
            audio.hapticBuf = (int8_t*)getBufferForSubPacket(subPacketTypeHaptic);
            if (audio.hapticBuf == nullptr) {
                LOGE("getBufferForSubPacket for haptic failed");
                break;
            }
        }

        audio.hapticBuf[audio.hapticBufPos++] = outL;
        audio.hapticBuf[audio.hapticBufPos++] = outR;

        if (audio.hapticBufPos == subPacketHapticSize) {
            // 如果是第一次发送一个静音audio包，避免haptic包等待
            if (audio.needSendFirstMuteOpusPackage) {
                audio.needSendFirstMuteOpusPackage = false;
                uint8_t* audioBuff = getBufferForSubPacket(subPacketTypeAudio);
                if (audioBuff != nullptr) {
                    setMuteOpusPackage(audioBuff);
                    writeSubPacket(audioBuff, subPacketTypeAudio);
                } else {
                    LOGW("getBufferForSubPacket failed");
                }
            }

            writeSubPacket((uint8_t*)audio.hapticBuf, subPacketTypeHaptic);
            audio.hapticBuf = nullptr;
            audio.hapticBufPos = 0;
        }
    }
}

uint8_t count = 0;
static int16_t opusInBuf[audioOpusInFrames * audioChannels];
int16_t* resampleOutBuf = opusInBuf;

static inline void __not_in_flash_func(speakerProc)() {
    audio.audioWaitData = true;
    struct AudioRawElement* audioRawElement = nullptr;
    if (!queue_try_remove(&audio.audioPcmFifo, (void*)&audioRawElement)) {
        return;
    }

    if (!config.audioActive) {
        lerpRsReset(&audio.audioResampler);
        freeAudioRawElement(audioRawElement);
        audioRawElement = nullptr;
        resampleOutBuf = opusInBuf;
        count = 0;
        return;
    }
    audio.audioWaitData = false;

    // 将 audioResamplerInputFrames frames 重采样成 audioResamplerOutputFrames frames 以解决噪音问题。感谢 @Junhoo
    const int outFrames = lerpRsProcess(&audio.audioResampler, audioRawElement->data, audioResamplerInputFrames, resampleOutBuf, audioResamplerOutputFrames);
    freeAudioRawElement(audioRawElement);
    audioRawElement = nullptr;
    if (outFrames != audioResamplerOutputFrames) {
        LOGE("ResampleOut failed, outFrames:%d", outFrames);
    }
    resampleOutBuf += audioResamplerOutputFrames * audioChannels;

    if (++count < audioResamplerOutToOpusInCount) {
        return;
    }

    resampleOutBuf = opusInBuf;
    count = 0;

    uint8_t* audioOut = getBufferForSubPacket(subPacketTypeAudio);
    if (audioOut == nullptr) {
        LOGW("Opus get out buf err");
        return;
    }
    const int encodedBytes = opus_encode(audio.encoder, opusInBuf, audioOpusInFrames, audioOut, subPacketAudioSize);
    if (encodedBytes < 0) {
        LOGE("opus_encode_float failed:%d", encodedBytes);
        freeSubPacket(audioOut, subPacketTypeAudio);
        return;
    }
    if (config.audioActive) {
        if (encodedBytes < subPacketAudioSize) {
            memset(audioOut + encodedBytes, 0, subPacketAudioSize - encodedBytes);
        }
        writeSubPacket(audioOut, subPacketTypeAudio);
    } else {
        freeSubPacket(audioOut, subPacketTypeAudio);
    }
}

// Mic path: opus packets from the controller (core0 mic_fifo) -> opus decode ->
// PCM into mic_decode_fifo for audio_loop to push to the USB IN endpoint.
static void __not_in_flash_func(micProc)() {
    struct MicOpuselement* opusElement = nullptr;
    if (!queue_try_remove(&audio.micOpusFifo, (void*)&opusElement)) {
        return;
    }

    struct MicPcmElement* pcmElement = getMicPcmElement();
    if (pcmElement == nullptr) {
        LOGE("pcmElement is nullptr");
        return;
    }

    pcmElement->frames = opus_decode(audio.decoder, opusElement->data, micOpusSize, pcmElement->data, micFrames, false);
    freeMicOpusElement(opusElement);
    if (pcmElement->frames <= 0) {
        // Gated behind ENABLE_VERBOSE: printf pulls the newlib formatting chain
        // (flash) onto core1's path. Release builds compile it out so core1's
        // audio loop stays fully RAM-resident (no XIP fetches on this core).
        LOGE("[Audio] OpusDecoder decode failed: %d", pcmElement->frames);
        freeMicPcmElement(pcmElement);
        return;
    }

    if (!queue_try_add(&audio.micPcmFifo, (void*)&pcmElement)) {
        freeMicPcmElement(pcmElement);
        LOGE("micPcmFifo: queue add failed");
    }
}

void __not_in_flash_func(core1Entry)() {
    // Register core1 as a flash-safe victim so core0's flash_safe_execute()
    // (config_save) actually parks this core while flash is erased/programmed,
    // instead of letting it fault on XIP. Requires PICO_FLASH_ASSUME_CORE1_SAFE=0.
    flash_safe_execute_core_init();

    for (;;) {
        speakerProc();
        micProc();
    }
}

// data points at the opus mic payload, len is the bytes available there.
// In RAM (consistent with the BT-receive path) and validates len so a short
// or malformed report can't over-read past the packet buffer.
void __not_in_flash_func(micAddOpusQueue)(uint8_t* data, uint16_t len) {
    if (len < micOpusSize) {
        return;
    }

    struct MicOpuselement* opusElement = getMicOpusElement();
    if (opusElement == nullptr) {
        LOGE("no free micOpusElement");
        return;
    }

    memcpy(opusElement->data, data, micOpusSize);

    if (!queue_try_add(&audio.micOpusFifo, (void*)&opusElement)) {
        freeMicOpusElement(opusElement);
        LOGE("micOpusFifo: queue add failed");
    }
}

void audioInit() {
    lerpRsInit(&audio.audioResampler, 51200, 48000, audioChannels);

    // Mic queues are read from audio_loop on core0 every iteration, so they
    // must exist regardless of the speaker-proc build flag.
    queue_init(&audio.micOpusFifo, sizeof(struct MicOpuselement*), micOpusElementSize);
    queue_init(&audio.micPcmFifo, sizeof(struct MicPcmElement*), micPcmElementSize);

    for (int i = 0; i < audioRawElementSize; ++i) {
        atomic_init(&audio.audioRawElementArray[i].inuse, false);
    }

    for (int i = 0; i < micOpusElementSize; ++i) {
        atomic_init(&audio.micOpusElementArray[i].inuse, false);
    }

    for (int i = 0; i < micPcmElementSize; ++i) {
        atomic_init(&audio.micPcmElementArray[i].inuse, false);
    }

    audio.hapticBuf = nullptr;
    audio.currentAudioRawElement = nullptr;
    audio.rawPos = 0;
    audio.hapticBufPos = 0;
    audio.audioBufPos = 0;
    audio.needClean = false;
    audio.needSendFirstMuteOpusPackage = true;
    audio.audioWaitData = false;

    int error = 0;
    // RESTRICTED_LOWDELAY: 强制纯 CELT，跳过 SILK/Hybrid 模式决策和 look-ahead 分析
    audio.encoder = opus_encoder_create(48000, audioChannels, OPUS_APPLICATION_RESTRICTED_LOWDELAY, &error);
    if (error != 0) {
        LOGE("OpusEncoder create failed");
        return;
    }
    opus_encoder_ctl(audio.encoder, OPUS_SET_EXPERT_FRAME_DURATION(OPUS_FRAMESIZE_10_MS));
    // 码率 = 200字节 × 8bit × 100fps = 160kbps，确保每帧 CBR 输出恰好 200 字节以匹配协议
    opus_encoder_ctl(audio.encoder, OPUS_SET_BITRATE(200 * 8 * 100));
    opus_encoder_ctl(audio.encoder, OPUS_SET_VBR(false));
    opus_encoder_ctl(audio.encoder, OPUS_SET_COMPLEXITY(0));  // 范围 0-10，0 最低复杂度
    // 告知编码器输入精度为 int16
    opus_encoder_ctl(audio.encoder, OPUS_SET_LSB_DEPTH(16));
    // 禁用帧间预测
    opus_encoder_ctl(audio.encoder, OPUS_SET_PREDICTION_DISABLED(1));

    audio.decoder = opus_decoder_create(48000, micChannels, &error);
    if (error != 0) {
        LOGE("[Audio] OpusDecoder create failed");
    }

    queue_init(&audio.audioPcmFifo, sizeof(struct AudioRawElement*), audioRawElementSize);
    multicore_launch_core1_with_stack(core1Entry, audio.audio_core1_stack, sizeof(audio.audio_core1_stack));
}
