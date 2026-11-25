#pragma once

#include <cstdint>

namespace Drive {

enum class Wheel : uint8_t {
    FL = 0,
    FR = 1,
    RR = 2,
    RL = 3
};

struct EncoderSnapshot {
    int64_t rawCounts;          // Counter direction as physically wired.
    int64_t forwardCounts;      // Sign-normalized: positive means wheel forward.
    int64_t rotationsMilli;     // Wheel revolutions, scaled by 1000.
    int64_t distanceMilliMm;    // Distance in millimetres, scaled by 1000.
    int16_t commandPercent;     // Signed command: + forward, - reverse.
};

// Initializes TIM1/TIM8 drive PWM and TIM2/TIM3/TIM4/TIM5 quadrature encoders.
// The caller must configure the system clock before calling this function.
bool Init();

// Signed speed command in the range -100..+100. Positive is physical forward.
void SetWheelSpeed(Wheel wheel, int16_t percent);
void SetAllWheelSpeeds(int16_t fl, int16_t fr, int16_t rr, int16_t rl);
void StopAll();

// Poll hardware encoder counters; call frequently from the main loop.
void Update();

bool GetEncoderSnapshot(Wheel wheel, EncoderSnapshot *snapshot);
int16_t GetWheelCommand(Wheel wheel);

} // namespace Drive
