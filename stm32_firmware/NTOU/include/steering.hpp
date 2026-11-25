#pragma once

#include <cstdint>

namespace Steering {

enum class Axis : uint8_t {
    SR = 0,
    SL = 1
};

struct AxisSnapshot {
    uint16_t raw;
    int32_t angleCdeg;
    int32_t positionCdeg;   // Positive RIGHT, negative LEFT, relative to centre.
    int32_t errorCdeg;
    uint16_t magnitude;
    uint8_t status;
    bool sensorValid;
    bool holdover;
    int8_t motorDirection;  // +1 RIGHT, -1 LEFT, 0 STOP.
    uint8_t pwmPercent;
};

// Initializes steering PWM and I2C1/TCA9548A. Requires system clock first.
bool Init();

// Absolute steering target relative to the calibrated centre, -40..+40 deg.
bool SetTargetDegrees(int16_t degrees);
void DisableHold();

// Call frequently from the main loop. Internally runs the control loop at 50 Hz.
void Update(uint32_t nowMs);

bool GetAxisSnapshot(Axis axis, AxisSnapshot *snapshot);
int16_t GetTargetDegrees();
bool IsHolding();
bool IsTcaDetected();

} // namespace Steering
