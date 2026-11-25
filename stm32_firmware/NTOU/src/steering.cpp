#include "steering.hpp"
#include "stm32f4xx_hal.h"

#include <cmath>
#include <cstdint>
#include <cstdlib>

namespace Steering {
namespace {

constexpr uint32_t kPwmPeriod = 799U;             // 20 kHz at 16 MHz timer clock
constexpr uint32_t kControlIntervalMs = 20U;      // 50 Hz
constexpr uint32_t kI2cTimeoutMs = 20U;
constexpr uint32_t kTcaSettleMs = 2U;
constexpr uint32_t kSensorDropoutToleranceMs = 1500U;
constexpr uint32_t kTcaRetryIntervalMs = 1000U;

constexpr uint8_t kTcaAddress = 0x70U;
constexpr uint8_t kAs5600Address = 0x36U;
constexpr uint8_t kStatusRegister = 0x0BU;
constexpr uint8_t kRawAngleRegister = 0x0CU;
constexpr uint8_t kMagnitudeRegister = 0x1BU;
constexpr uint8_t kStatusMagnetDetected = 0x20U;
constexpr uint8_t kStatusMagnetWeak = 0x10U;
constexpr uint8_t kStatusMagnetStrong = 0x08U;

// Latest measured centres supplied by the user, not the older test-file values.
constexpr uint8_t kSrTcaChannel = 0U;
constexpr uint8_t kSlTcaChannel = 1U;
constexpr int32_t kSrCenterCdeg = 545;     // CH0: 5.45 degrees, raw about 62.
constexpr int32_t kSlCenterCdeg = 20804;   // CH1: 208.04 degrees, raw about 2367.
constexpr int8_t kSrPositionSign = +1;
constexpr int8_t kSlPositionSign = +1;

constexpr int32_t kTargetLimitCdeg = 4000;       // +/-40 degrees
constexpr int32_t kHoldDeadbandCdeg = 150;       // +/-1.50 degrees
constexpr float kKpPercentPerDegree = 1.8f;
constexpr float kKdPercentPerDegreePerSec = 0.25f;
constexpr uint8_t kMinPwmPercent = 50U;
constexpr uint8_t kMaxPwmPercent = 80U;          // Matches the latest successful test.

TIM_HandleTypeDef g_tim12 = {};
TIM_HandleTypeDef g_tim13 = {};
TIM_HandleTypeDef g_tim14 = {};
I2C_HandleTypeDef g_i2c1 = {};

struct AxisState {
    uint8_t channel;
    int32_t centerCdeg;
    int8_t positionSign;
    uint16_t raw;
    int32_t angleCdeg;
    int32_t positionCdeg;
    int32_t errorCdeg;
    uint16_t magnitude;
    uint8_t status;
    bool sensorValid;
    bool hasValidSample;
    bool hasPrevious;
    bool dropoutActive;
    bool holdover;
    uint32_t lastValidMs;
    uint32_t previousSampleMs;
    int32_t previousRawAngleCdeg;
    float velocityDegPerSec;
    int8_t motorDirection;
    uint8_t pwmPercent;
};

AxisState g_sr = {
    kSrTcaChannel, kSrCenterCdeg, kSrPositionSign,
    0U, 0, 0, 0, 0U, 0U, false, false, false, false, false,
    0U, 0U, 0, 0.0f, 0, 0U
};
AxisState g_sl = {
    kSlTcaChannel, kSlCenterCdeg, kSlPositionSign,
    0U, 0, 0, 0, 0U, 0U, false, false, false, false, false,
    0U, 0U, 0, 0.0f, 0, 0U
};

bool g_initialized = false;
bool g_tcaDetected = false;
bool g_holdEnabled = false;
int16_t g_targetDegrees = 0;
uint32_t g_lastControlMs = 0U;
uint32_t g_lastTcaRetryMs = 0U;

uint32_t DutyCompare(uint8_t percent)
{
    return ((kPwmPeriod + 1U) * static_cast<uint32_t>(percent)) / 100U;
}

int32_t SignedAngleDifferenceCdeg(int32_t angleCdeg, int32_t referenceCdeg)
{
    int32_t difference = angleCdeg - referenceCdeg;
    while (difference > 18000) difference -= 36000;
    while (difference < -18000) difference += 36000;
    return difference;
}

bool ConfigurePwmTimer(TIM_HandleTypeDef *timer,
                       TIM_TypeDef *instance,
                       bool configureChannel2)
{
    timer->Instance = instance;
    timer->Init.Prescaler = 0U;
    timer->Init.CounterMode = TIM_COUNTERMODE_UP;
    timer->Init.Period = kPwmPeriod;
    timer->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    timer->Init.RepetitionCounter = 0U;
    timer->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_PWM_Init(timer) != HAL_OK) return false;

    TIM_OC_InitTypeDef channel = {};
    channel.OCMode = TIM_OCMODE_PWM1;
    channel.Pulse = 0U;
    channel.OCPolarity = TIM_OCPOLARITY_HIGH;
    channel.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(timer, &channel, TIM_CHANNEL_1) != HAL_OK) return false;
    if (configureChannel2 &&
        HAL_TIM_PWM_ConfigChannel(timer, &channel, TIM_CHANNEL_2) != HAL_OK) return false;
    return true;
}

void InitSteeringPins()
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitTypeDef gpio = {};
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_PULLDOWN;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;

    gpio.Pin = GPIO_PIN_6 | GPIO_PIN_7;
    HAL_GPIO_Init(GPIOA, &gpio);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6 | GPIO_PIN_7, GPIO_PIN_RESET);

    gpio.Pin = GPIO_PIN_14 | GPIO_PIN_15;
    HAL_GPIO_Init(GPIOB, &gpio);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14 | GPIO_PIN_15, GPIO_PIN_RESET);

    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_PULLDOWN;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;

    gpio.Pin = GPIO_PIN_6;
    gpio.Alternate = GPIO_AF9_TIM13;
    HAL_GPIO_Init(GPIOA, &gpio);

    gpio.Pin = GPIO_PIN_7;
    gpio.Alternate = GPIO_AF9_TIM14;
    HAL_GPIO_Init(GPIOA, &gpio);

    // SL motor direction was reversed after the IBT-2 replacement:
    // RIGHT=PB15/TIM12_CH2, LEFT=PB14/TIM12_CH1.
    gpio.Pin = GPIO_PIN_14 | GPIO_PIN_15;
    gpio.Alternate = GPIO_AF9_TIM12;
    HAL_GPIO_Init(GPIOB, &gpio);
}

bool InitI2c1()
{
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_I2C1_CLK_ENABLE();

    GPIO_InitTypeDef gpio = {};
    gpio.Pin = GPIO_PIN_8 | GPIO_PIN_9;
    gpio.Mode = GPIO_MODE_AF_OD;
    gpio.Pull = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB, &gpio);

    g_i2c1.Instance = I2C1;
    g_i2c1.Init.ClockSpeed = 100000U;
    g_i2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
    g_i2c1.Init.OwnAddress1 = 0U;
    g_i2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    g_i2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    g_i2c1.Init.OwnAddress2 = 0U;
    g_i2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    g_i2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    return HAL_I2C_Init(&g_i2c1) == HAL_OK;
}

bool FindTca()
{
    if (HAL_I2C_IsDeviceReady(&g_i2c1,
                              static_cast<uint16_t>(kTcaAddress << 1U),
                              2U, kI2cTimeoutMs) == HAL_OK) {
        g_tcaDetected = true;
        return true;
    }
    g_tcaDetected = false;
    return false;
}

bool SelectTcaChannel(uint8_t channel)
{
    if (!g_tcaDetected || channel > 7U) return false;
    uint8_t mask = static_cast<uint8_t>(1U << channel);
    if (HAL_I2C_Master_Transmit(&g_i2c1,
                                static_cast<uint16_t>(kTcaAddress << 1U),
                                &mask, 1U, kI2cTimeoutMs) != HAL_OK) {
        return false;
    }
    HAL_Delay(kTcaSettleMs);
    return true;
}

bool ReadSensor(AxisState *axis)
{
    if (axis == nullptr || !SelectTcaChannel(axis->channel)) {
        if (axis != nullptr) axis->sensorValid = false;
        return false;
    }

    uint8_t status = 0U;
    uint8_t rawBytes[2] = {0U, 0U};
    uint8_t magnitudeBytes[2] = {0U, 0U};

    if (HAL_I2C_Mem_Read(&g_i2c1,
                         static_cast<uint16_t>(kAs5600Address << 1U),
                         kStatusRegister, I2C_MEMADD_SIZE_8BIT,
                         &status, 1U, kI2cTimeoutMs) != HAL_OK ||
        HAL_I2C_Mem_Read(&g_i2c1,
                         static_cast<uint16_t>(kAs5600Address << 1U),
                         kRawAngleRegister, I2C_MEMADD_SIZE_8BIT,
                         rawBytes, 2U, kI2cTimeoutMs) != HAL_OK ||
        HAL_I2C_Mem_Read(&g_i2c1,
                         static_cast<uint16_t>(kAs5600Address << 1U),
                         kMagnitudeRegister, I2C_MEMADD_SIZE_8BIT,
                         magnitudeBytes, 2U, kI2cTimeoutMs) != HAL_OK) {
        axis->sensorValid = false;
        return false;
    }

    axis->status = status;
    axis->raw = static_cast<uint16_t>(
        ((static_cast<uint16_t>(rawBytes[0]) & 0x0FU) << 8U) |
        static_cast<uint16_t>(rawBytes[1]));
    axis->angleCdeg = static_cast<int32_t>(
        (static_cast<uint32_t>(axis->raw) * 36000U + 2048U) / 4096U);
    axis->magnitude = static_cast<uint16_t>(
        ((static_cast<uint16_t>(magnitudeBytes[0]) & 0x0FU) << 8U) |
        static_cast<uint16_t>(magnitudeBytes[1]));

    const bool magnetDetected = (status & kStatusMagnetDetected) != 0U;
    const bool magnetWeak = (status & kStatusMagnetWeak) != 0U;
    const bool magnetStrong = (status & kStatusMagnetStrong) != 0U;
    if (!magnetDetected || magnetWeak || magnetStrong || axis->magnitude < 10U) {
        axis->sensorValid = false;
        return false;
    }

    const int32_t rawPosition = SignedAngleDifferenceCdeg(axis->angleCdeg,
                                                           axis->centerCdeg);
    axis->positionCdeg = rawPosition * axis->positionSign;
    axis->sensorValid = true;
    return true;
}

void StopSrMotor(AxisState *axis)
{
    __HAL_TIM_SET_COMPARE(&g_tim13, TIM_CHANNEL_1, 0U); // SR RIGHT PA6
    __HAL_TIM_SET_COMPARE(&g_tim14, TIM_CHANNEL_1, 0U); // SR LEFT PA7
    axis->motorDirection = 0;
    axis->pwmPercent = 0U;
}

void StopSlMotor(AxisState *axis)
{
    __HAL_TIM_SET_COMPARE(&g_tim12, TIM_CHANNEL_2, 0U); // SL RIGHT PB15
    __HAL_TIM_SET_COMPARE(&g_tim12, TIM_CHANNEL_1, 0U); // SL LEFT PB14
    axis->motorDirection = 0;
    axis->pwmPercent = 0U;
}

void StopAxisMotor(AxisState *axis)
{
    if (axis == &g_sr) StopSrMotor(axis);
    else if (axis == &g_sl) StopSlMotor(axis);
}

void StopAllSteeringMotors()
{
    StopSrMotor(&g_sr);
    StopSlMotor(&g_sl);
}

void SetSrMotor(int8_t direction, uint8_t pwmPercent)
{
    __HAL_TIM_SET_COMPARE(&g_tim13, TIM_CHANNEL_1, 0U);
    __HAL_TIM_SET_COMPARE(&g_tim14, TIM_CHANNEL_1, 0U);
    if (direction > 0) {
        __HAL_TIM_SET_COMPARE(&g_tim13, TIM_CHANNEL_1, DutyCompare(pwmPercent)); // RIGHT PA6
    } else if (direction < 0) {
        __HAL_TIM_SET_COMPARE(&g_tim14, TIM_CHANNEL_1, DutyCompare(pwmPercent)); // LEFT PA7
    }
    g_sr.motorDirection = direction;
    g_sr.pwmPercent = direction == 0 ? 0U : pwmPercent;
}

void SetSlMotor(int8_t direction, uint8_t pwmPercent)
{
    __HAL_TIM_SET_COMPARE(&g_tim12, TIM_CHANNEL_2, 0U); // PB15
    __HAL_TIM_SET_COMPARE(&g_tim12, TIM_CHANNEL_1, 0U); // PB14
    if (direction > 0) {
        __HAL_TIM_SET_COMPARE(&g_tim12, TIM_CHANNEL_2, DutyCompare(pwmPercent)); // RIGHT
    } else if (direction < 0) {
        __HAL_TIM_SET_COMPARE(&g_tim12, TIM_CHANNEL_1, DutyCompare(pwmPercent)); // LEFT
    }
    g_sl.motorDirection = direction;
    g_sl.pwmPercent = direction == 0 ? 0U : pwmPercent;
}

uint8_t CalculateDuty(int32_t errorCdeg, float velocityDegPerSec)
{
    const float errorDeg = static_cast<float>(std::abs(errorCdeg)) / 100.0f;
    const float directionToTarget = errorCdeg >= 0 ? 1.0f : -1.0f;
    const float approachVelocity = directionToTarget * velocityDegPerSec;
    const float damping = approachVelocity > 0.0f
        ? kKdPercentPerDegreePerSec * approachVelocity : 0.0f;
    float duty = kKpPercentPerDegree * errorDeg - damping;

    if (duty < static_cast<float>(kMinPwmPercent)) duty = static_cast<float>(kMinPwmPercent);
    if (duty > static_cast<float>(kMaxPwmPercent)) duty = static_cast<float>(kMaxPwmPercent);
    return static_cast<uint8_t>(duty + 0.5f);
}

void ApplyAxisController(AxisState *axis)
{
    if (axis == nullptr || !axis->sensorValid || !g_holdEnabled) {
        if (axis != nullptr) StopAxisMotor(axis);
        return;
    }

    const int32_t targetCdeg = static_cast<int32_t>(g_targetDegrees) * 100;
    axis->errorCdeg = targetCdeg - axis->positionCdeg;
    if (std::abs(axis->errorCdeg) <= kHoldDeadbandCdeg) {
        StopAxisMotor(axis);
        return;
    }

    const int8_t direction = axis->errorCdeg > 0 ? +1 : -1;
    const uint8_t duty = CalculateDuty(axis->errorCdeg, axis->velocityDegPerSec);
    if (axis == &g_sr) SetSrMotor(direction, duty);
    else SetSlMotor(direction, duty);
}

void UpdateAxis(AxisState *axis, bool readOk, uint32_t nowMs)
{
    if (readOk) {
        if (!axis->hasPrevious || axis->dropoutActive) {
            axis->velocityDegPerSec = 0.0f;
        } else {
            const uint32_t elapsedMs = nowMs - axis->previousSampleMs;
            if (elapsedMs > 0U) {
                int32_t deltaCdeg = SignedAngleDifferenceCdeg(axis->angleCdeg,
                                                               axis->previousRawAngleCdeg);
                deltaCdeg *= axis->positionSign;
                axis->velocityDegPerSec = static_cast<float>(deltaCdeg) /
                    (100.0f * (static_cast<float>(elapsedMs) / 1000.0f));
            }
        }

        axis->previousRawAngleCdeg = axis->angleCdeg;
        axis->previousSampleMs = nowMs;
        axis->hasPrevious = true;
        axis->hasValidSample = true;
        axis->lastValidMs = nowMs;
        axis->dropoutActive = false;
        axis->holdover = false;
        axis->errorCdeg = static_cast<int32_t>(g_targetDegrees) * 100 - axis->positionCdeg;
        ApplyAxisController(axis);
        return;
    }

    axis->sensorValid = false;
    if (!axis->dropoutActive) axis->dropoutActive = true;

    // User-requested 1.5 s tolerance: retain the last motor direction/PWM briefly.
    // Closed-loop control resumes automatically on the first valid sample.
    if (g_holdEnabled && axis->hasValidSample &&
        (nowMs - axis->lastValidMs) <= kSensorDropoutToleranceMs) {
        axis->holdover = true;
        return; // Intentionally leave the last compare value unchanged.
    }

    axis->holdover = false;
    StopAxisMotor(axis);
}

} // namespace

bool Init()
{
    g_initialized = false;
    InitSteeringPins();
    if (!InitI2c1()) return false;

    __HAL_RCC_TIM12_CLK_ENABLE();
    __HAL_RCC_TIM13_CLK_ENABLE();
    __HAL_RCC_TIM14_CLK_ENABLE();

    if (!ConfigurePwmTimer(&g_tim12, TIM12, true) ||
        !ConfigurePwmTimer(&g_tim13, TIM13, false) ||
        !ConfigurePwmTimer(&g_tim14, TIM14, false)) {
        return false;
    }

    if (HAL_TIM_PWM_Start(&g_tim12, TIM_CHANNEL_1) != HAL_OK ||
        HAL_TIM_PWM_Start(&g_tim12, TIM_CHANNEL_2) != HAL_OK ||
        HAL_TIM_PWM_Start(&g_tim13, TIM_CHANNEL_1) != HAL_OK ||
        HAL_TIM_PWM_Start(&g_tim14, TIM_CHANNEL_1) != HAL_OK) {
        return false;
    }

    StopAllSteeringMotors();
    g_holdEnabled = false;
    g_targetDegrees = 0;
    g_lastControlMs = HAL_GetTick();
    g_lastTcaRetryMs = HAL_GetTick();
    g_tcaDetected = FindTca(); // A missing mux is retried from Update().
    g_initialized = true;
    return true;
}

bool SetTargetDegrees(int16_t degrees)
{
    if (!g_initialized || degrees < -40 || degrees > 40) return false;
    g_targetDegrees = degrees;
    g_holdEnabled = true;
    return true;
}

void DisableHold()
{
    g_holdEnabled = false;
    StopAllSteeringMotors();
}

void Update(uint32_t nowMs)
{
    if (!g_initialized || (nowMs - g_lastControlMs) < kControlIntervalMs) return;
    g_lastControlMs = nowMs;

    if (!g_tcaDetected && (nowMs - g_lastTcaRetryMs) >= kTcaRetryIntervalMs) {
        g_lastTcaRetryMs = nowMs;
        FindTca();
    }

    const bool srOk = ReadSensor(&g_sr);
    const bool slOk = ReadSensor(&g_sl);
    UpdateAxis(&g_sr, srOk, nowMs);
    UpdateAxis(&g_sl, slOk, nowMs);
}

bool GetAxisSnapshot(Axis axis, AxisSnapshot *snapshot)
{
    if (snapshot == nullptr || !g_initialized) return false;
    const AxisState *state = nullptr;
    if (axis == Axis::SR) state = &g_sr;
    else if (axis == Axis::SL) state = &g_sl;
    else return false;

    snapshot->raw = state->raw;
    snapshot->angleCdeg = state->angleCdeg;
    snapshot->positionCdeg = state->positionCdeg;
    snapshot->errorCdeg = state->errorCdeg;
    snapshot->magnitude = state->magnitude;
    snapshot->status = state->status;
    snapshot->sensorValid = state->sensorValid;
    snapshot->holdover = state->holdover;
    snapshot->motorDirection = state->motorDirection;
    snapshot->pwmPercent = state->pwmPercent;
    return true;
}

int16_t GetTargetDegrees() { return g_targetDegrees; }
bool IsHolding() { return g_holdEnabled; }
bool IsTcaDetected() { return g_tcaDetected; }

} // namespace Steering
