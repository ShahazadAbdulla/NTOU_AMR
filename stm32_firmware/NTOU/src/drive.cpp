#include "drive.hpp"
#include "stm32f4xx_hal.h"

#include <cstdint>

namespace Drive {
namespace {

constexpr uint32_t kPwmPeriod = 799U;       // 16 MHz / 800 = 20 kHz
constexpr int64_t kCountsPerRevolution = 608972LL; // Historical; calibration pending.
constexpr int64_t kCircumferenceMilliMm = 628319LL; // 200 mm wheel, mm * 1000; provisional.
constexpr uint8_t kEncoderFilter = 8U;

TIM_HandleTypeDef g_tim1 = {};
TIM_HandleTypeDef g_tim8 = {};
TIM_HandleTypeDef g_tim2 = {};
TIM_HandleTypeDef g_tim3 = {};
TIM_HandleTypeDef g_tim4 = {};
TIM_HandleTypeDef g_tim5 = {};

struct MotorChannelMap {
    TIM_HandleTypeDef *timer;
    uint32_t forwardChannel;
    uint32_t reverseChannel;
};

// Physical-forward mapping verified during the individual motor tests.
// FR and RR are deliberately direction-swapped relative to RPWM/LPWM labels.
MotorChannelMap g_motorMap[4] = {
    {&g_tim1, TIM_CHANNEL_1, TIM_CHANNEL_2}, // FL: PA8 forward, PA9 reverse
    {&g_tim1, TIM_CHANNEL_4, TIM_CHANNEL_3}, // FR: PA11 forward, PA10 reverse
    {&g_tim8, TIM_CHANNEL_4, TIM_CHANNEL_3}, // RR: PC9 forward, PC8 reverse
    {&g_tim8, TIM_CHANNEL_1, TIM_CHANNEL_2}  // RL: PC6 forward, PC7 reverse
};

struct EncoderState {
    TIM_HandleTypeDef *timer;
    uint32_t previousCounter;
    int64_t rawCounts;
    int8_t forwardSign;
    bool is32Bit;
};

// Array order is FL, FR, RR, RL.
EncoderState g_encoders[4] = {
    {&g_tim5, 0U, 0LL, +1, true},  // FL: PA0/PA1, TIM5
    {&g_tim3, 0U, 0LL, -1, false}, // FR: PB4/PB5, TIM3
    {&g_tim2, 0U, 0LL, -1, true},  // RR: PA15/PB3, TIM2
    {&g_tim4, 0U, 0LL, +1, false} // RL: PB6/PB7, TIM4
};

int16_t g_commands[4] = {0, 0, 0, 0};
bool g_initialized = false;

bool IsValidWheel(Wheel wheel)
{
    return static_cast<uint8_t>(wheel) < 4U;
}

uint32_t DutyCompare(int16_t percent)
{
    const uint32_t magnitude = static_cast<uint32_t>(percent < 0 ? -percent : percent);
    return ((kPwmPeriod + 1U) * magnitude) / 100U;
}

bool ConfigurePwmTimer(TIM_HandleTypeDef *timer, TIM_TypeDef *instance)
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

    return HAL_TIM_PWM_ConfigChannel(timer, &channel, TIM_CHANNEL_1) == HAL_OK &&
           HAL_TIM_PWM_ConfigChannel(timer, &channel, TIM_CHANNEL_2) == HAL_OK &&
           HAL_TIM_PWM_ConfigChannel(timer, &channel, TIM_CHANNEL_3) == HAL_OK &&
           HAL_TIM_PWM_ConfigChannel(timer, &channel, TIM_CHANNEL_4) == HAL_OK;
}

bool ConfigureEncoderTimer(TIM_HandleTypeDef *timer,
                           TIM_TypeDef *instance,
                           uint32_t period)
{
    timer->Instance = instance;
    timer->Init.Prescaler = 0U;
    timer->Init.CounterMode = TIM_COUNTERMODE_UP;
    timer->Init.Period = period;
    timer->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    timer->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    TIM_Encoder_InitTypeDef encoder = {};
    encoder.EncoderMode = TIM_ENCODERMODE_TI12; // Both edges of A and B (x4).
    encoder.IC1Polarity = TIM_ICPOLARITY_RISING;
    encoder.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    encoder.IC1Prescaler = TIM_ICPSC_DIV1;
    encoder.IC1Filter = kEncoderFilter;
    encoder.IC2Polarity = TIM_ICPOLARITY_RISING;
    encoder.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    encoder.IC2Prescaler = TIM_ICPSC_DIV1;
    encoder.IC2Filter = kEncoderFilter;

    if (HAL_TIM_Encoder_Init(timer, &encoder) != HAL_OK) return false;
    __HAL_TIM_SET_COUNTER(timer, 0U);
    return HAL_TIM_Encoder_Start(timer, TIM_CHANNEL_ALL) == HAL_OK;
}

void InitDrivePwmPins()
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    GPIO_InitTypeDef gpio = {};
    gpio.Pin = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_PULLDOWN;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &gpio);
    HAL_GPIO_WritePin(GPIOA,
                      GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11,
                      GPIO_PIN_RESET);

    gpio.Pin = GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9;
    HAL_GPIO_Init(GPIOC, &gpio);
    HAL_GPIO_WritePin(GPIOC,
                      GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9,
                      GPIO_PIN_RESET);

    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_PULLDOWN;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;

    gpio.Pin = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11;
    gpio.Alternate = GPIO_AF1_TIM1;
    HAL_GPIO_Init(GPIOA, &gpio);

    gpio.Pin = GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9;
    gpio.Alternate = GPIO_AF3_TIM8;
    HAL_GPIO_Init(GPIOC, &gpio);
}

void InitEncoderPins()
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitTypeDef gpio = {};
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;

    // FL: PA0/PA1 -> TIM5 CH1/CH2, AF2.
    gpio.Pin = GPIO_PIN_0 | GPIO_PIN_1;
    gpio.Alternate = GPIO_AF2_TIM5;
    HAL_GPIO_Init(GPIOA, &gpio);

    // RR: PA15/PB3 -> TIM2 CH1/CH2, AF1.
    // On F446, selecting these timer AFs on the GPIO pins releases the JTAG use;
    // SWD remains on PA13/PA14.
    gpio.Pin = GPIO_PIN_15;
    gpio.Alternate = GPIO_AF1_TIM2;
    HAL_GPIO_Init(GPIOA, &gpio);
    gpio.Pin = GPIO_PIN_3;
    HAL_GPIO_Init(GPIOB, &gpio);

    // FR: PB4/PB5 -> TIM3 CH1/CH2, AF2.
    gpio.Pin = GPIO_PIN_4 | GPIO_PIN_5;
    gpio.Alternate = GPIO_AF2_TIM3;
    HAL_GPIO_Init(GPIOB, &gpio);

    // RL: PB6/PB7 -> TIM4 CH1/CH2, AF2.
    gpio.Pin = GPIO_PIN_6 | GPIO_PIN_7;
    gpio.Alternate = GPIO_AF2_TIM4;
    HAL_GPIO_Init(GPIOB, &gpio);
}

int64_t CounterDelta32(uint32_t current, uint32_t previous)
{
    return static_cast<int32_t>(current - previous);
}

int64_t CounterDelta16(uint32_t current, uint32_t previous)
{
    return static_cast<int16_t>(static_cast<uint16_t>(current - previous));
}

int64_t ScaleByRatio(int64_t counts, int64_t numerator, int64_t denominator)
{
    // Split quotient/remainder to avoid multiplying the full accumulated count.
    const int64_t whole = counts / denominator;
    const int64_t remainder = counts % denominator;
    return whole * numerator + (remainder * numerator) / denominator;
}

} // namespace

bool Init()
{
    g_initialized = false;
    InitDrivePwmPins();

    __HAL_RCC_TIM1_CLK_ENABLE();
    __HAL_RCC_TIM8_CLK_ENABLE();
    if (!ConfigurePwmTimer(&g_tim1, TIM1) || !ConfigurePwmTimer(&g_tim8, TIM8)) {
        return false;
    }

    if (HAL_TIM_PWM_Start(&g_tim1, TIM_CHANNEL_1) != HAL_OK ||
        HAL_TIM_PWM_Start(&g_tim1, TIM_CHANNEL_2) != HAL_OK ||
        HAL_TIM_PWM_Start(&g_tim1, TIM_CHANNEL_3) != HAL_OK ||
        HAL_TIM_PWM_Start(&g_tim1, TIM_CHANNEL_4) != HAL_OK ||
        HAL_TIM_PWM_Start(&g_tim8, TIM_CHANNEL_1) != HAL_OK ||
        HAL_TIM_PWM_Start(&g_tim8, TIM_CHANNEL_2) != HAL_OK ||
        HAL_TIM_PWM_Start(&g_tim8, TIM_CHANNEL_3) != HAL_OK ||
        HAL_TIM_PWM_Start(&g_tim8, TIM_CHANNEL_4) != HAL_OK) {
        return false;
    }

    // TIM1 and TIM8 are advanced-control timers; enable their main output gates.
    __HAL_TIM_MOE_ENABLE(&g_tim1);
    __HAL_TIM_MOE_ENABLE(&g_tim8);
    StopAll();

    __HAL_RCC_TIM2_CLK_ENABLE();
    __HAL_RCC_TIM3_CLK_ENABLE();
    __HAL_RCC_TIM4_CLK_ENABLE();
    __HAL_RCC_TIM5_CLK_ENABLE();
    InitEncoderPins();

    if (!ConfigureEncoderTimer(&g_tim5, TIM5, 0xFFFFFFFFU) ||
        !ConfigureEncoderTimer(&g_tim2, TIM2, 0xFFFFFFFFU) ||
        !ConfigureEncoderTimer(&g_tim3, TIM3, 0xFFFFU) ||
        !ConfigureEncoderTimer(&g_tim4, TIM4, 0xFFFFU)) {
        StopAll();
        return false;
    }

    for (uint8_t i = 0U; i < 4U; ++i) {
        g_encoders[i].previousCounter = __HAL_TIM_GET_COUNTER(g_encoders[i].timer);
        g_encoders[i].rawCounts = 0LL;
        g_commands[i] = 0;
    }

    g_initialized = true;
    return true;
}

void SetWheelSpeed(Wheel wheel, int16_t percent)
{
    if (!g_initialized || !IsValidWheel(wheel)) return;
    if (percent > 100) percent = 100;
    if (percent < -100) percent = -100;

    const uint8_t index = static_cast<uint8_t>(wheel);
    MotorChannelMap &map = g_motorMap[index];

    // Break-before-make: clear both direction inputs before setting one channel.
    __HAL_TIM_SET_COMPARE(map.timer, map.forwardChannel, 0U);
    __HAL_TIM_SET_COMPARE(map.timer, map.reverseChannel, 0U);

    if (percent > 0) {
        __HAL_TIM_SET_COMPARE(map.timer, map.forwardChannel, DutyCompare(percent));
    } else if (percent < 0) {
        __HAL_TIM_SET_COMPARE(map.timer, map.reverseChannel, DutyCompare(percent));
    }

    g_commands[index] = percent;
}

void SetAllWheelSpeeds(int16_t fl, int16_t fr, int16_t rr, int16_t rl)
{
    SetWheelSpeed(Wheel::FL, fl);
    SetWheelSpeed(Wheel::FR, fr);
    SetWheelSpeed(Wheel::RR, rr);
    SetWheelSpeed(Wheel::RL, rl);
}

void StopAll()
{
    if (g_tim1.Instance != nullptr) {
        __HAL_TIM_SET_COMPARE(&g_tim1, TIM_CHANNEL_1, 0U);
        __HAL_TIM_SET_COMPARE(&g_tim1, TIM_CHANNEL_2, 0U);
        __HAL_TIM_SET_COMPARE(&g_tim1, TIM_CHANNEL_3, 0U);
        __HAL_TIM_SET_COMPARE(&g_tim1, TIM_CHANNEL_4, 0U);
    }
    if (g_tim8.Instance != nullptr) {
        __HAL_TIM_SET_COMPARE(&g_tim8, TIM_CHANNEL_1, 0U);
        __HAL_TIM_SET_COMPARE(&g_tim8, TIM_CHANNEL_2, 0U);
        __HAL_TIM_SET_COMPARE(&g_tim8, TIM_CHANNEL_3, 0U);
        __HAL_TIM_SET_COMPARE(&g_tim8, TIM_CHANNEL_4, 0U);
    }
    for (uint8_t i = 0U; i < 4U; ++i) g_commands[i] = 0;
}

void Update()
{
    if (!g_initialized) return;

    for (uint8_t i = 0U; i < 4U; ++i) {
        const uint32_t current = __HAL_TIM_GET_COUNTER(g_encoders[i].timer);
        const int64_t delta = g_encoders[i].is32Bit
            ? CounterDelta32(current, g_encoders[i].previousCounter)
            : CounterDelta16(current, g_encoders[i].previousCounter);
        g_encoders[i].rawCounts += delta;
        g_encoders[i].previousCounter = current;
    }
}

bool GetEncoderSnapshot(Wheel wheel, EncoderSnapshot *snapshot)
{
    if (!g_initialized || !IsValidWheel(wheel) || snapshot == nullptr) return false;

    const uint8_t index = static_cast<uint8_t>(wheel);
    const int64_t raw = g_encoders[index].rawCounts;
    const int64_t forward = raw * static_cast<int64_t>(g_encoders[index].forwardSign);

    snapshot->rawCounts = raw;
    snapshot->forwardCounts = forward;
    snapshot->rotationsMilli = ScaleByRatio(forward, 1000LL, kCountsPerRevolution);
    snapshot->distanceMilliMm = ScaleByRatio(forward, kCircumferenceMilliMm, kCountsPerRevolution);
    snapshot->commandPercent = g_commands[index];
    return true;
}

int16_t GetWheelCommand(Wheel wheel)
{
    if (!IsValidWheel(wheel)) return 0;
    return g_commands[static_cast<uint8_t>(wheel)];
}

} // namespace Drive
