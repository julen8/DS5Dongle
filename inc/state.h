#pragma once

#include <stdint.h>

constexpr int ds5StatePacketSize = 63;
constexpr int ds5ControlPacketSize = 63;
constexpr int ds5ControlPayloadSize = 47;

enum PowerState : uint8_t {
    PowerStateDischarging = 0x00,          // Use PowerPercent
    PowerStateCharging = 0x01,             // Use PowerPercent
    PowerStateComplete = 0x02,             // PowerPercent not valid? assume 100%?
    PowerStateAbnormalVoltage = 0x0A,      // PowerPercent not valid?
    PowerStateAbnormalTemperature = 0x0B,  // PowerPercent not valid?
    PowerStateChargingError = 0x0F         // PowerPercent not valid?
};

enum Direction : uint8_t {
    DirectionNorth = 0,
    DirectionNorthEast = 1,
    DirectionEast = 2,
    DirectionSouthEast = 3,
    DirectionSouth = 4,
    DirectionSouthWest = 5,
    DirectionWest = 6,
    DirectionNorthWest = 7,
    DirectionNone = 8,
};

struct __attribute__((packed)) TouchFingerData {  // 4
    /*0.0*/ uint32_t Index : 7;
    /*0.7*/ uint32_t NotTouching : 1;
    /*1.0*/ uint32_t FingerX : 12;
    /*2.4*/ uint32_t FingerY : 12;
};

struct __attribute__((packed)) TouchData {  // 9
    /*0*/ struct TouchFingerData Finger[2];
    /*8*/ uint8_t Timestamp;
};

// DS5(BT) -> PC(USB)
struct __attribute__((packed)) Ds5StatePacket {  // 63
    /* 0  */ uint8_t LeftStickX;
    /* 1  */ uint8_t LeftStickY;
    /* 2  */ uint8_t RightStickX;
    /* 3  */ uint8_t RightStickY;
    /* 4  */ uint8_t TriggerLeft;
    /* 5  */ uint8_t TriggerRight;
    /* 6  */ uint8_t SeqNo;  // always 0x01 on BT
    /* 7.0*/ enum Direction DPad : 4;
    /* 7.4*/ uint8_t ButtonSquare : 1;
    /* 7.5*/ uint8_t ButtonCross : 1;
    /* 7.6*/ uint8_t ButtonCircle : 1;
    /* 7.7*/ uint8_t ButtonTriangle : 1;
    /* 8.0*/ uint8_t ButtonL1 : 1;
    /* 8.1*/ uint8_t ButtonR1 : 1;
    /* 8.2*/ uint8_t ButtonL2 : 1;
    /* 8.3*/ uint8_t ButtonR2 : 1;
    /* 8.4*/ uint8_t ButtonCreate : 1;
    /* 8.5*/ uint8_t ButtonOptions : 1;
    /* 8.6*/ uint8_t ButtonL3 : 1;
    /* 8.7*/ uint8_t ButtonR3 : 1;
    /* 9.0*/ uint8_t ButtonHome : 1;
    /* 9.1*/ uint8_t ButtonPad : 1;
    /* 9.2*/ uint8_t ButtonMute : 1;
    /* 9.3*/ uint8_t UNK1 : 1;                 // appears unused
    /* 9.4*/ uint8_t ButtonLeftFunction : 1;   // DualSense Edge
    /* 9.5*/ uint8_t ButtonRightFunction : 1;  // DualSense Edge
    /* 9.6*/ uint8_t ButtonLeftPaddle : 1;     // DualSense Edge
    /* 9.7*/ uint8_t ButtonRightPaddle : 1;    // DualSense Edge
    /*10  */ uint8_t UNK2;                     // appears unused
    /*11  */ uint32_t UNK_COUNTER;             // Linux driver calls this reserved, tools leak calls the 2 high bytes "random"
    /*15  */ int16_t AngularVelocityX;
    /*17  */ int16_t AngularVelocityZ;
    /*19  */ int16_t AngularVelocityY;
    /*21  */ int16_t AccelerometerX;
    /*23  */ int16_t AccelerometerY;
    /*25  */ int16_t AccelerometerZ;
    /*27  */ uint32_t SensorTimestamp;
    /*31  */ int8_t Temperature;  // reserved2 in Linux driver
    /*32  */ struct TouchData Touch;
    /*41.0*/ uint8_t TriggerRightStopLocation : 4;  // trigger stop can be a range from 0 to 9 (F/9.0 for Apple interface)
    /*41.4*/ uint8_t TriggerRightStatus : 4;
    /*42.0*/ uint8_t TriggerLeftStopLocation : 4;
    /*42.4*/ uint8_t TriggerLeftStatus : 4;   // 0 feedbackNoLoad
                                              // 1 feedbackLoadApplied
                                              // 0 weaponReady
                                              // 1 weaponFiring
                                              // 2 weaponFired
                                              // 0 vibrationNotVibrating
                                              // 1 vibrationIsVibrating
    /*43  */ uint32_t HostTimestamp;          // mirrors data from report write
    /*47.0*/ uint8_t TriggerRightEffect : 4;  // Active trigger effect, previously we thought this was status max
    /*47.4*/ uint8_t TriggerLeftEffect : 4;   // 0 for reset and all other effects
                                              // 1 for feedback effect
                                              // 2 for weapon effect
                                              // 3 for vibration
    /*48  */ uint32_t DeviceTimeStamp;
    /*52.0*/ uint8_t PowerPercent : 4;  // 0x00-0x0A
    /*52.4*/ enum PowerState Power : 4;
    /*53.0*/ uint8_t PluggedHeadphones : 1;
    /*53.1*/ uint8_t PluggedMic : 1;
    /*53.2*/ uint8_t MicMuted : 1;  // Mic muted by powersave/mute command
    /*53.3*/ uint8_t PluggedUsbData : 1;
    /*53.4*/ uint8_t PluggedUsbPower : 1;  // appears that this cannot be 1 if PluggedUsbData is 1
    /*53.5*/ uint8_t UsbPowerOnBT : 1;     // appears this is only 1 if BT connected and USB powered
    /*53.5*/ uint8_t DockDetect : 1;
    /*53.5*/ uint8_t PluggedUnk : 1;
    /*54.0*/ uint8_t PluggedExternalMic : 1;   // Is external mic active (automatic in mic auto mode)
    /*54.1*/ uint8_t HapticLowPassFilter : 1;  // Is the Haptic Low-Pass-Filter active?
    /*54.2*/ uint8_t PluggedUnk3 : 6;
    /*55  */ uint8_t AesCmac[8];
};

enum MuteLight : uint8_t {
    MuteLightOff = 0,
    MuteLightOn = 1,
    MuteLightBreathing = 2,
    MuteLightDoNothing = 3,  // literally nothing, this input is ignored,
                             // though it might be a faster blink in other versions
    MuteLightNoAction4 = 4,
    MuteLightNoAction5 = 5,
    MuteLightNoAction6 = 6,
    MuteLightNoAction7 = 7,
};

enum LightBrightness : uint8_t {
    LightBrightnessBright = 0,
    LightBrightnessMid = 1,
    LightBrightnessDim = 2,
    LightBrightnessNoAction3 = 3,
    LightBrightnessNoAction4 = 4,
    LightBrightnessNoAction5 = 5,
    LightBrightnessNoAction6 = 6,
    LightBrightnessNoAction7 = 7,
};

enum LightFadeAnimation : uint8_t {
    LightFadeAnimationNothing = 0,
    LightFadeAnimationFadeIn = 1,   // from black to blue
    LightFadeAnimationFadeOut = 2,  // from blue to black
};

// PC(USB) -> DS5(BT)
struct __attribute__((packed)) Ds5ControlPacket {   // 63 字节 47字节有效载荷
    /*    */                                        // Report Set Flags
    /*    */                                        // These flags are used to indicate what contents from this report should be processed
    /* 0.0*/ uint8_t EnableRumbleEmulation : 1;     // Suggest halving rumble strength
    /* 0.1*/ uint8_t UseRumbleNotHaptics : 1;       //
                                                    /*    */
    /* 0.2*/ uint8_t AllowRightTriggerFFB : 1;      // Enable setting RightTriggerFFB
    /* 0.3*/ uint8_t AllowLeftTriggerFFB : 1;       // Enable setting LeftTriggerFFB
                                                    /*    */
    /* 0.4*/ uint8_t AllowHeadphoneVolume : 1;      // Enable setting VolumeHeadphones
    /* 0.5*/ uint8_t AllowSpeakerVolume : 1;        // Enable setting VolumeSpeaker
    /* 0.6*/ uint8_t AllowMicVolume : 1;            // Enable setting VolumeMic
                                                    /*    */
    /* 0.7*/ uint8_t AllowAudioControl : 1;         // Enable setting AudioControl section
    /* 1.0*/ uint8_t AllowMuteLight : 1;            // Enable setting MuteLightMode
    /* 1.1*/ uint8_t AllowAudioMute : 1;            // Enable setting MuteControl section
                                                    /*    */
    /* 1.2*/ uint8_t AllowLedColor : 1;             // Enable RGB LED section
                                                    /*    */
    /* 1.3*/ uint8_t ResetLights : 1;               // Release the LEDs from Wireless firmware control
    /*    */                                        // When in wireless mode this must be signaled to control LEDs
    /*    */                                        // This cannot be applied during the BT pair animation.
    /*    */                                        // SDL2 waits until the SensorTimestamp value is >= 10200000
    /*    */                                        // before pulsing this bit once.
    /*    */                                        //
    /* 1.4*/ uint8_t AllowPlayerIndicators : 1;     // Enable setting PlayerIndicators section
    /* 1.5*/ uint8_t AllowHapticLowPassFilter : 1;  // Enable HapticLowPassFilter
    /* 1.6*/ uint8_t AllowMotorPowerLevel : 1;      // MotorPowerLevel reductions for trigger/haptic
    /* 1.7*/ uint8_t AllowAudioControl2 : 1;        // Enable setting AudioControl2 section
    /*    */                                        //
    /* 2  */ uint8_t RumbleEmulationRight;          // emulates the light weight
    /* 3  */ uint8_t RumbleEmulationLeft;           // emulated the heavy weight
    /*    */                                        //
    /* 4  */ uint8_t VolumeHeadphones;              // max 0x7f
    /* 5  */ uint8_t VolumeSpeaker;                 // PS5 appears to only use the range 0x3d-0x64
    /* 6  */ uint8_t VolumeMic;                     // not linear, seems to max at 64, 0 is fully muted only in chat mode
    /*    */                                        //
    /*    */                                        // AudioControl
    /* 7.0*/ uint8_t MicSelect : 2;                 // 0 Auto
    /*    */                                        // 1 Internal Only
    /*    */                                        // 2 External Only
    /*    */                                        // 3 Unclear, sets external mic flag but might use internal mic, do test
    /* 7.2*/ uint8_t EchoCancelEnable : 1;
    /* 7.3*/ uint8_t NoiseCancelEnable : 1;
    /* 7.4*/ uint8_t OutputPathSelect : 2;  // 0 L_R_X
    /*    */                                // 1 L_L_X
    /*    */                                // 2 L_L_R
    /*    */                                // 3 X_X_R
    /* 7.6*/ uint8_t InputPathSelect : 2;   // 0 CHAT_ASR
    /*    */                                // 1 CHAT_CHAT
    /*    */                                // 2 ASR_ASR
    /*    */                                // 3 Does Nothing, invalid
    /*    */                                //
    /* 8  */ enum MuteLight MuteLightMode;
    /*    */
    /*    */  // MuteControl
    /* 9.0*/ uint8_t TouchPowerSave : 1;
    /* 9.1*/ uint8_t MotionPowerSave : 1;
    /* 9.2*/ uint8_t HapticPowerSave : 1;  // AKA BulletPowerSave
    /* 9.3*/ uint8_t AudioPowerSave : 1;
    /* 9.4*/ uint8_t MicMute : 1;
    /* 9.5*/ uint8_t SpeakerMute : 1;
    /* 9.6*/ uint8_t HeadphoneMute : 1;
    /* 9.7*/ uint8_t HapticMute : 1;  // AKA BulletMute
    /*    */                          //
    /*10  */ uint8_t RightTriggerFFB[11];
    /*21  */ uint8_t LeftTriggerFFB[11];
    /*32  */ uint32_t HostTimestamp;                     // mirrored into report read
    /*    */                                             //
    /*    */                                             // MotorPowerLevel
    /*36.0*/ uint8_t RumbleMotorPowerReduction : 4;      // 0x0-0x7 (no 0x8?) Applied in 12.5% reductions
    /*36.4*/ uint8_t TriggerMotorPowerReduction : 4;     // 0x0-0xA
    /*    */                                             //
    /*    */                                             // AudioControl2
    /*37.0*/ uint8_t SpeakerCompPreGain : 3;             // additional speaker volume boost
    /*37.3*/ uint8_t BeamformingEnable : 1;              // Probably for MIC given there's 2, might be more bits, can't find what it does
    /*37.4*/ uint8_t UnkAudioControl2 : 4;               // some of these bits might apply to the above
    /*    */                                             //
    /*38.0*/ uint8_t AllowLightBrightnessChange : 1;     // LED_BRIHTNESS_CONTROL
    /*38.1*/ uint8_t AllowColorLightFadeAnimation : 1;   // LIGHTBAR_SETUP_CONTROL
    /*38.2*/ uint8_t EnableImprovedRumbleEmulation : 1;  // Use instead of EnableRumbleEmulation
                                                         // requires FW >= 0x0224
                                                         // No need to halve rumble strength
    /*38.3*/ uint8_t UseRumbleNotHaptics2 : 1;           // 在 NinjaGaiden4 出现，与 UseRumbleNotHaptics 工作效果相同
    /*38.4*/ uint8_t UNKBITC : 4;                        // unused
    /*    */                                             //
    /*39.0*/ uint8_t HapticLowPassFilter : 1;
    /*39.1*/ uint8_t UNKBIT : 7;
    /*    */
    /*40  */ uint8_t UNKBYTE;  // previous notes suggested this was HLPF, was probably off by 1
    /*    */                   //
    /*41  */ enum LightFadeAnimation LightFadeAnimation;
    /*42  */ enum LightBrightness LightBrightness;
    /*    */
    /*    */                               // PlayerIndicators
    /*    */                               // These bits control the white LEDs under the touch pad.
    /*    */                               // Note the reduction in functionality for later revisions.
    /*    */                               // Generation 0x03 - Full Functionality
    /*    */                               // Generation 0x04 - Mirrored Only
    /*    */                               // Suggested detection: (HardwareInfo & 0x00FFFF00) == 0X00000400
    /*    */                               //
    /*    */                               // Layout used by PS5:
    /*    */                               // 0x04 - -x- -  Player 1
    /*    */                               // 0x06 - x-x -  Player 2
    /*    */                               // 0x15 x -x- x  Player 3
    /*    */                               // 0x1B x x-x x  Player 4
    /*    */                               // 0x1F x xxx x  Player 5* (Unconfirmed)
    /*    */                               //
    /*    */                               //                        // HW 0x03 // HW 0x04
    /*43.0*/ uint8_t PlayerLight1 : 1;     // x --- - // x --- x
    /*43.1*/ uint8_t PlayerLight2 : 1;     // - x-- - // - x-x -
    /*43.2*/ uint8_t PlayerLight3 : 1;     // - -x- - // - -x- -
    /*43.3*/ uint8_t PlayerLight4 : 1;     // - --x - // - x-x -
    /*43.4*/ uint8_t PlayerLight5 : 1;     // - --- x // x --- x
    /*43.5*/ uint8_t PlayerLightFade : 1;  // if low player lights fade in, if high player lights instantly change
    /*43.6*/ uint8_t PlayerLightUNK : 2;
    /*    */
    /*    */  // RGB LED
    /*44  */ uint8_t LedRed;
    /*45  */ uint8_t LedGreen;
    /*46  */ uint8_t LedBlue;
    uint8_t reserved[16];
    // Structure ends here though on BT there is padding and a CRC, see ReportOut31
};

union Ds5StateUnion {
    uint8_t data[ds5StatePacketSize];
    struct Ds5StatePacket packet;
};

union Ds5ControlUnion {
    uint8_t data[ds5ControlPacketSize];
    struct Ds5ControlPacket packet;
};

static constexpr union Ds5ControlUnion ds5ControlInitPacket = {
    .packet =
        {
            .EnableRumbleEmulation = 1,
            .UseRumbleNotHaptics = 0,
            .AllowRightTriggerFFB = 1,
            .AllowLeftTriggerFFB = 1,
            .AllowHeadphoneVolume = 1,
            .AllowSpeakerVolume = 1,
            .AllowMicVolume = 1,
            .AllowAudioControl = 1,

            .AllowMuteLight = 1,
            .AllowAudioMute = 1,
            .AllowLedColor = 1,
            .ResetLights = 0,
            .AllowPlayerIndicators = 1,
            .AllowHapticLowPassFilter = 1,
            .AllowMotorPowerLevel = 1,
            .AllowAudioControl2 = 1,

            .RumbleEmulationRight = 0,
            .RumbleEmulationLeft = 0,
            .VolumeHeadphones = 0x50,
            .VolumeSpeaker = 0x50,
            .VolumeMic = 0x64,

            .MicSelect = 2,
            .EchoCancelEnable = 1,
            .NoiseCancelEnable = 1,
            .OutputPathSelect = 0,
            .InputPathSelect = 0,

            .MuteLightMode = MuteLightOff,

            .TouchPowerSave = 1,
            .MotionPowerSave = 1,
            .HapticPowerSave = 1,
            .AudioPowerSave = 1,
            .MicMute = 0,
            .SpeakerMute = 0,
            .HeadphoneMute = 0,
            .HapticMute = 0,

            .RightTriggerFFB = {0},
            .LeftTriggerFFB = {0},
            .HostTimestamp = 0,

            .RumbleMotorPowerReduction = 0,
            .TriggerMotorPowerReduction = 0,

            .SpeakerCompPreGain = 2,
            .BeamformingEnable = 1,
            .UnkAudioControl2 = 0,

            .AllowLightBrightnessChange = 1,
            .AllowColorLightFadeAnimation = 1,
            .EnableImprovedRumbleEmulation = 1,
            .UseRumbleNotHaptics2 = 0,
            .UNKBITC = 0,

            .HapticLowPassFilter = 0,
            .UNKBIT = 0,

            .UNKBYTE = 0,
            .LightFadeAnimation = LightFadeAnimationFadeOut,
            .LightBrightness = LightBrightnessMid,

            .PlayerLight1 = 0,
            .PlayerLight2 = 0,
            .PlayerLight3 = 0,
            .PlayerLight4 = 0,
            .PlayerLight5 = 0,
            .PlayerLightFade = 0,
            .PlayerLightUNK = 0,

            .LedRed = 0xff,
            .LedGreen = 0xd7,
            .LedBlue = 0,
            .reserved = {0},
        },
};

void setStatePacket(const union Ds5StateUnion *packet);
union Ds5StateUnion *getStatePacket();

void setControlPacket(const uint8_t *data, int size);
union Ds5ControlUnion *getControlPacket();
void reSendControlPacket();

void updateVolume();
void updateMicVolume();
