#include "stm32f4xx_hal.h"
#include <cstdint>
#include <cstdio>
#include <cstring>

// STM32F446RE - four drive motors + four incremental wheel encoders.
// Steering motors and steering sensors are deliberately not configured.
// USART2 PA2/PA3 is the Nucleo ST-LINK virtual COM port.
//
// Drive PWM pins:
//   FL: PA8/PA9   TIM1 CH1/CH2
//   FR: PA10/PA11 TIM1 CH3/CH4 (right-side polarity is swapped in software)
//   RR: PC8/PC9   TIM8 CH3/CH4 (right-side polarity is swapped in software)
//   RL: PC6/PC7   TIM8 CH1/CH2
//
// Encoder pins (hardware timer quadrature mode TI12, x4):
//   FL: PA0/PA1   TIM5 CH1/CH2 (32-bit)
//   RR: PA15/PB3  TIM2 CH1/CH2 (32-bit)
//   FR: PB4/PB5   TIM3 CH1/CH2 (16-bit)
//   RL: PB6/PB7   TIM4 CH1/CH2 (16-bit)
//
// Historical mechanical scale: 608,972 timer counts per wheel revolution,
// wheel diameter 200 mm, circumference 628.319 mm. This scale is PROVISIONAL
// on the current F446RE TI12/x4 configuration and must be verified by rotating
// each wheel exactly one revolution before using the distance as odometry.

UART_HandleTypeDef huart2;
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;

static constexpr uint32_t PWM_PERIOD = 799U;             // 16 MHz / 800 = 20 kHz
static constexpr uint32_t PWM_DUTY_50 = 400U;            // 50% duty
static constexpr uint32_t TEST_DURATION_MS = 10000U;     // 10 seconds
static constexpr uint32_t TELEMETRY_INTERVAL_MS = 250U;
static constexpr int64_t COUNTS_PER_WHEEL_REV = 608972LL; // provisional historical value
static constexpr int64_t WHEEL_CIRCUMFERENCE_UM = 628319LL;

static bool testActive = false;
static uint32_t testStartMs = 0U;
static uint32_t lastTelemetryMs = 0U;
static char commandBuffer[32] = {0};
static uint8_t commandLength = 0U;

struct EncoderState {
    TIM_HandleTypeDef *timer;
    uint32_t previousCounter;
    int64_t totalCounts;
    bool is32Bit;
    const char *name;
};

// Array order is FL, FR, RR, RL to match the user-facing motor labels.
static EncoderState encoders[4] = {
    {&htim5, 0U, 0LL, true,  "FL"},
    {&htim3, 0U, 0LL, false, "FR"},
    {&htim2, 0U, 0LL, true,  "RR"},
    {&htim4, 0U, 0LL, false, "RL"}
};

static void Error_Handler(void);
static void SystemClock_Config(void);
static void GPIO_Init(void);
static void USART2_Init(void);
static void TIM1_PWM_Init(void);
static void TIM8_PWM_Init(void);
static void Encoder_GPIO_Init(void);
static void Encoder_Timer_Init(TIM_HandleTypeDef *timer, TIM_TypeDef *instance, uint32_t period);
static void Encoder_Init_All(void);
static void Encoder_Update_All(void);
static void Encoder_Reset_All(void);
static void UART_Print(const char *text);
static void StopAllMotors(void);
static void SetMotor(uint8_t motorIndex, bool forward);
static void StartTestCommand(char command);
static void ProcessCommand(const char *command);
static void ProcessSerial(void);
static void PrintTelemetry(const char *prefix);
static void PrintFinalSummary(void);

extern "C" void SysTick_Handler(void)
{
    HAL_IncTick();
    HAL_SYSTICK_IRQHandler();
}

static void Error_Handler(void)
{
    // PWM compare values are initialized to zero before outputs are started.
    __disable_irq();
    while (1) {
    }
}

static void SystemClock_Config(void)
{
    // Keep the Nucleo on the internal 16 MHz HSI, with APB1/APB2 undivided.
    // All four encoder timers and TIM1/TIM8 therefore receive 16 MHz timer clocks.
    RCC_OscInitTypeDef osc = {};
    RCC_ClkInitTypeDef clock = {};

    osc.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    osc.HSIState = RCC_HSI_ON;
    osc.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    osc.PLL.PLLState = RCC_PLL_NONE;
    if (HAL_RCC_OscConfig(&osc) != HAL_OK) {
        Error_Handler();
    }

    clock.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                      RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    clock.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
    clock.AHBCLKDivider = RCC_SYSCLK_DIV1;
    clock.APB1CLKDivider = RCC_HCLK_DIV1;
    clock.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&clock, FLASH_LATENCY_0) != HAL_OK) {
        Error_Handler();
    }
}

static void GPIO_Init(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    GPIO_InitTypeDef gpio = {};

    // First hold all eight drive PWM pins low as GPIO outputs.
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    gpio.Pin = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11;
    HAL_GPIO_Init(GPIOA, &gpio);
    HAL_GPIO_WritePin(GPIOA, gpio.Pin, GPIO_PIN_RESET);

    gpio.Pin = GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9;
    HAL_GPIO_Init(GPIOC, &gpio);
    HAL_GPIO_WritePin(GPIOC, gpio.Pin, GPIO_PIN_RESET);

    // TIM1: PA8..PA11, AF1, channels 1..4.
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = GPIO_AF1_TIM1;
    gpio.Pin = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11;
    HAL_GPIO_Init(GPIOA, &gpio);

    // TIM8: PC6..PC9, AF3, channels 1..4.
    gpio.Alternate = GPIO_AF3_TIM8;
    gpio.Pin = GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9;
    HAL_GPIO_Init(GPIOC, &gpio);
}

static void USART2_Init(void)
{
    __HAL_RCC_USART2_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitTypeDef gpio = {};
    gpio.Pin = GPIO_PIN_2 | GPIO_PIN_3;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &gpio);

    huart2.Instance = USART2;
    huart2.Init.BaudRate = 115200U;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart2) != HAL_OK) {
        Error_Handler();
    }
}

static void ConfigurePwmTimer(TIM_HandleTypeDef *timer, TIM_TypeDef *instance)
{
    timer->Instance = instance;
    timer->Init.Prescaler = 0U;
    timer->Init.CounterMode = TIM_COUNTERMODE_UP;
    timer->Init.Period = PWM_PERIOD;
    timer->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    timer->Init.RepetitionCounter = 0U;
    timer->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    if (HAL_TIM_PWM_Init(timer) != HAL_OK) {
        Error_Handler();
    }

    TIM_OC_InitTypeDef channel = {};
    channel.OCMode = TIM_OCMODE_PWM1;
    channel.Pulse = 0U;
    channel.OCPolarity = TIM_OCPOLARITY_HIGH;
    channel.OCFastMode = TIM_OCFAST_DISABLE;

    if (HAL_TIM_PWM_ConfigChannel(timer, &channel, TIM_CHANNEL_1) != HAL_OK ||
        HAL_TIM_PWM_ConfigChannel(timer, &channel, TIM_CHANNEL_2) != HAL_OK ||
        HAL_TIM_PWM_ConfigChannel(timer, &channel, TIM_CHANNEL_3) != HAL_OK ||
        HAL_TIM_PWM_ConfigChannel(timer, &channel, TIM_CHANNEL_4) != HAL_OK) {
        Error_Handler();
    }
}

static void TIM1_PWM_Init(void)
{
    __HAL_RCC_TIM1_CLK_ENABLE();
    ConfigurePwmTimer(&htim1, TIM1);
}

static void TIM8_PWM_Init(void)
{
    __HAL_RCC_TIM8_CLK_ENABLE();
    ConfigurePwmTimer(&htim8, TIM8);
}

static void Encoder_GPIO_Init(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitTypeDef gpio = {};
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;

    // FL encoder: PA0/PA1 -> TIM5 CH1/CH2, AF2.
    gpio.Pin = GPIO_PIN_0 | GPIO_PIN_1;
    gpio.Alternate = GPIO_AF2_TIM5;
    HAL_GPIO_Init(GPIOA, &gpio);

    // RR encoder: PA15 -> TIM2 CH1, AF1.
    // PA15 is a JTAG-only pin; configuring its MODER/AFR for TIM2 releases it.
    gpio.Pin = GPIO_PIN_15;
    gpio.Alternate = GPIO_AF1_TIM2;
    HAL_GPIO_Init(GPIOA, &gpio);

    // RR encoder: PB3 -> TIM2 CH2, AF1. PB3 is also JTAG-only.
    gpio.Pin = GPIO_PIN_3;
    gpio.Alternate = GPIO_AF1_TIM2;
    HAL_GPIO_Init(GPIOB, &gpio);

    // FR encoder: PB4/PB5 -> TIM3 CH1/CH2, AF2. PB4 is JTAG-only.
    gpio.Pin = GPIO_PIN_4 | GPIO_PIN_5;
    gpio.Alternate = GPIO_AF2_TIM3;
    HAL_GPIO_Init(GPIOB, &gpio);

    // RL encoder: PB6/PB7 -> TIM4 CH1/CH2, AF2.
    gpio.Pin = GPIO_PIN_6 | GPIO_PIN_7;
    gpio.Alternate = GPIO_AF2_TIM4;
    HAL_GPIO_Init(GPIOB, &gpio);
}

static void Encoder_Timer_Init(TIM_HandleTypeDef *timer,
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
    encoder.EncoderMode = TIM_ENCODERMODE_TI12; // quadrature x4 counting
    encoder.IC1Polarity = TIM_ICPOLARITY_RISING;
    encoder.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    encoder.IC1Prescaler = TIM_ICPSC_DIV1;
    encoder.IC1Filter = 8U;
    encoder.IC2Polarity = TIM_ICPOLARITY_RISING;
    encoder.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    encoder.IC2Prescaler = TIM_ICPSC_DIV1;
    encoder.IC2Filter = 8U;

    if (HAL_TIM_Encoder_Init(timer, &encoder) != HAL_OK) {
        Error_Handler();
    }
    __HAL_TIM_SET_COUNTER(timer, 0U);
    if (HAL_TIM_Encoder_Start(timer, TIM_CHANNEL_ALL) != HAL_OK) {
        Error_Handler();
    }
}

static void Encoder_Init_All(void)
{
    __HAL_RCC_TIM2_CLK_ENABLE();
    __HAL_RCC_TIM3_CLK_ENABLE();
    __HAL_RCC_TIM4_CLK_ENABLE();
    __HAL_RCC_TIM5_CLK_ENABLE();

    Encoder_GPIO_Init();

    Encoder_Timer_Init(&htim5, TIM5, 0xFFFFFFFFU); // FL, 32-bit
    Encoder_Timer_Init(&htim2, TIM2, 0xFFFFFFFFU); // RR, 32-bit
    Encoder_Timer_Init(&htim3, TIM3, 0xFFFFU);     // FR, 16-bit
    Encoder_Timer_Init(&htim4, TIM4, 0xFFFFU);     // RL, 16-bit

    for (uint8_t i = 0U; i < 4U; ++i) {
        encoders[i].previousCounter = __HAL_TIM_GET_COUNTER(encoders[i].timer);
        encoders[i].totalCounts = 0LL;
    }
}

static int64_t CounterDelta32(uint32_t current, uint32_t previous)
{
    const uint32_t difference = current - previous; // unsigned subtraction wraps safely
    if (difference <= 0x7FFFFFFFU) {
        return static_cast<int64_t>(difference);
    }
    return static_cast<int64_t>(difference) - 4294967296LL;
}

static int64_t CounterDelta16(uint32_t current, uint32_t previous)
{
    const uint16_t difference = static_cast<uint16_t>(current - previous);
    if (difference <= 0x7FFFU) {
        return static_cast<int64_t>(difference);
    }
    return static_cast<int64_t>(difference) - 65536LL;
}

static void Encoder_Update_All(void)
{
    for (uint8_t i = 0U; i < 4U; ++i) {
        const uint32_t current = __HAL_TIM_GET_COUNTER(encoders[i].timer);
        const int64_t delta = encoders[i].is32Bit
                                  ? CounterDelta32(current, encoders[i].previousCounter)
                                  : CounterDelta16(current, encoders[i].previousCounter);
        encoders[i].totalCounts +=
            ((i == 1U || i == 2U) ? -delta : delta);
        encoders[i].previousCounter = current;
    }
}

static void Encoder_Reset_All(void)
{
    // Called at the beginning of a test, before motor PWM is enabled.
    for (uint8_t i = 0U; i < 4U; ++i) {
        __HAL_TIM_SET_COUNTER(encoders[i].timer, 0U);
        encoders[i].previousCounter = 0U;
        encoders[i].totalCounts = 0LL;
    }
}

static void UART_Print(const char *text)
{
    if (text != nullptr) {
        HAL_UART_Transmit(&huart2,
                          reinterpret_cast<uint8_t *>(const_cast<char *>(text)),
                          static_cast<uint16_t>(std::strlen(text)),
                          100U);
    }
}

static void StopAllMotors(void)
{
    // TIM1: FL CH1/CH2 and FR CH3/CH4.
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0U);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0U);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0U);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0U);

    // TIM8: RL CH1/CH2 and RR CH3/CH4.
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, 0U);
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, 0U);
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, 0U);
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, 0U);
}

// Motor index: 0=FL, 1=FR, 2=RR, 3=RL.
// The FR and RR RPWM/LPWM assignments are intentionally swapped because
// the previous physical test showed their forward/reverse directions inverted.
static void SetMotor(uint8_t motorIndex, bool forward)
{
    TIM_HandleTypeDef *timer = nullptr;
    uint32_t forwardChannel = 0U;
    uint32_t reverseChannel = 0U;

    switch (motorIndex) {
        case 0U: // FL: PA8 RPWM / PA9 LPWM, TIM1 CH1/CH2
            timer = &htim1;
            forwardChannel = TIM_CHANNEL_1;
            reverseChannel = TIM_CHANNEL_2;
            break;
        case 1U: // FR right side: PA11 is forward, PA10 is reverse
            timer = &htim1;
            forwardChannel = TIM_CHANNEL_4;
            reverseChannel = TIM_CHANNEL_3;
            break;
        case 2U: // RR right side: PC9 is forward, PC8 is reverse
            timer = &htim8;
            forwardChannel = TIM_CHANNEL_4;
            reverseChannel = TIM_CHANNEL_3;
            break;
        case 3U: // RL: PC6 RPWM / PC7 LPWM, TIM8 CH1/CH2
            timer = &htim8;
            forwardChannel = TIM_CHANNEL_1;
            reverseChannel = TIM_CHANNEL_2;
            break;
        default:
            return;
    }

    // Break-before-make: both direction inputs are set to zero before one is enabled.
    __HAL_TIM_SET_COMPARE(timer, forwardChannel, 0U);
    __HAL_TIM_SET_COMPARE(timer, reverseChannel, 0U);
    if (forward) {
        __HAL_TIM_SET_COMPARE(timer, forwardChannel, PWM_DUTY_50);
    } else {
        __HAL_TIM_SET_COMPARE(timer, reverseChannel, PWM_DUTY_50);
    }
}

static void AppendText(char *destination, size_t capacity, const char *text)
{
    if (destination == nullptr || text == nullptr || capacity == 0U) return;
    const size_t used = std::strlen(destination);
    if (used >= capacity - 1U) return;
    std::strncat(destination, text, capacity - used - 1U);
}

static void AppendUnsigned64(char *destination, size_t capacity, uint64_t value)
{
    char digits[21];
    size_t count = 0U;
    do {
        digits[count++] = static_cast<char>('0' + (value % 10U));
        value /= 10U;
    } while (value != 0U && count < sizeof(digits));

    while (count > 0U) {
        char one[2] = {digits[--count], '\0'};
        AppendText(destination, capacity, one);
    }
}

static void AppendSigned64(char *destination, size_t capacity, int64_t value)
{
    if (value < 0) {
        AppendText(destination, capacity, "-");
        const uint64_t magnitude = static_cast<uint64_t>(-(value + 1LL)) + 1ULL;
        AppendUnsigned64(destination, capacity, magnitude);
    } else {
        AppendUnsigned64(destination, capacity, static_cast<uint64_t>(value));
    }
}

// Append a signed value expressed in thousandths, without using %lld.
// This avoids dependence on optional long-long printf support in newlib-nano.
static void AppendFixed3(char *destination, size_t capacity, int64_t valueMilli)
{
    const bool negative = (valueMilli < 0);
    const uint64_t magnitude = negative
        ? static_cast<uint64_t>(-(valueMilli + 1LL)) + 1ULL
        : static_cast<uint64_t>(valueMilli);
    if (negative) AppendText(destination, capacity, "-");
    AppendUnsigned64(destination, capacity, magnitude / 1000ULL);
    AppendText(destination, capacity, ".");
    const uint32_t fraction = static_cast<uint32_t>(magnitude % 1000ULL);
    char fractionalDigits[4] = {
        static_cast<char>('0' + (fraction / 100U)),
        static_cast<char>('0' + ((fraction / 10U) % 10U)),
        static_cast<char>('0' + (fraction % 10U)),
        '\0'
    };
    AppendText(destination, capacity, fractionalDigits);
}

static void AppendEncoderTelemetry(char *line, size_t lineSize, const EncoderState &encoder)
{
    // Quotient/remainder arithmetic avoids multiplying the full 64-bit count by the scale.
    const int64_t wholeRevolutions = encoder.totalCounts / COUNTS_PER_WHEEL_REV;
    const int64_t remainderCounts = encoder.totalCounts % COUNTS_PER_WHEEL_REV;
    const int64_t rotationsMilli =
        wholeRevolutions * 1000LL + (remainderCounts * 1000LL) / COUNTS_PER_WHEEL_REV;
    const int64_t distanceMicrometres =
        wholeRevolutions * WHEEL_CIRCUMFERENCE_UM +
        (remainderCounts * WHEEL_CIRCUMFERENCE_UM) / COUNTS_PER_WHEEL_REV;

    AppendText(line, lineSize, encoder.name);
    AppendText(line, lineSize, " cnt=");
    AppendSigned64(line, lineSize, encoder.totalCounts);
    AppendText(line, lineSize, " rot=");
    AppendFixed3(line, lineSize, rotationsMilli);
    AppendText(line, lineSize, " rev dist=");
    AppendFixed3(line, lineSize, distanceMicrometres);
    AppendText(line, lineSize, " mm");
}

static void PrintTelemetry(const char *prefix)
{
    Encoder_Update_All();

    char line[384] = {0};
    if (prefix != nullptr) AppendText(line, sizeof(line), prefix);

    AppendEncoderTelemetry(line, sizeof(line), encoders[0]);
    AppendText(line, sizeof(line), " | ");
    AppendEncoderTelemetry(line, sizeof(line), encoders[1]);
    AppendText(line, sizeof(line), " | ");
    AppendEncoderTelemetry(line, sizeof(line), encoders[2]);
    AppendText(line, sizeof(line), " | ");
    AppendEncoderTelemetry(line, sizeof(line), encoders[3]);
    AppendText(line, sizeof(line), "\r\n");
    UART_Print(line);
}

static void PrintFinalSummary(void)
{
    PrintTelemetry("FINAL | ");
    UART_Print("Test finished. All four drive motors stopped.\r\n");
}

static void StartTestCommand(char command)
{
    StopAllMotors();
    Encoder_Reset_All();

    switch (command) {
        case 'f':
            SetMotor(0U, true);
            SetMotor(1U, true);
            SetMotor(2U, true);
            SetMotor(3U, true);
            UART_Print("TEST: ALL wheels FORWARD at 50% PWM for 10 seconds.\r\n");
            break;
        case 'b':
            SetMotor(0U, false);
            SetMotor(1U, false);
            SetMotor(2U, false);
            SetMotor(3U, false);
            UART_Print("TEST: ALL wheels REVERSE at 50% PWM for 10 seconds.\r\n");
            break;
        case '1': SetMotor(0U, true);  UART_Print("TEST: FL FORWARD at 50% PWM for 10 seconds.\r\n"); break;
        case '2': SetMotor(0U, false); UART_Print("TEST: FL REVERSE at 50% PWM for 10 seconds.\r\n"); break;
        case '3': SetMotor(1U, true);  UART_Print("TEST: FR FORWARD via PA11 / TIM1_CH4 at 50% PWM for 10 seconds.\r\n"); break;
        case '4': SetMotor(1U, false); UART_Print("TEST: FR REVERSE via PA10 / TIM1_CH3 at 50% PWM for 10 seconds.\r\n"); break;
        case '5': SetMotor(2U, true);  UART_Print("TEST: RR FORWARD via PC9 / TIM8_CH4 at 50% PWM for 10 seconds.\r\n"); break;
        case '6': SetMotor(2U, false); UART_Print("TEST: RR REVERSE via PC8 / TIM8_CH3 at 50% PWM for 10 seconds.\r\n"); break;
        case '7': SetMotor(3U, true);  UART_Print("TEST: RL FORWARD via PC6 / TIM8_CH1 at 50% PWM for 10 seconds.\r\n"); break;
        case '8': SetMotor(3U, false); UART_Print("TEST: RL REVERSE via PC7 / TIM8_CH2 at 50% PWM for 10 seconds.\r\n"); break;
        default:
            return;
    }

    testStartMs = HAL_GetTick();
    lastTelemetryMs = testStartMs;
    testActive = true;
    PrintTelemetry("START | ");
}

static void ProcessCommand(const char *command)
{
    if (command == nullptr || command[0] == '\0') {
        return;
    }

    if (std::strlen(command) == 1U) {
        char c = command[0];
        if (c >= 'A' && c <= 'Z') {
            c = static_cast<char>(c - 'A' + 'a');
        }

        if (c == 's') {
            StopAllMotors();
            const bool wasActive = testActive;
            testActive = false;
            UART_Print("STOP: all drive motors stopped.\r\n");
            if (wasActive) {
                PrintFinalSummary();
            } else {
                PrintTelemetry("STATUS | ");
            }
            return;
        }

        if (c == 'f' || c == 'b' || (c >= '1' && c <= '8')) {
            StartTestCommand(c);
            return;
        }
    }

    if (std::strcmp(command, "help") == 0 || std::strcmp(command, "?") == 0) {
        UART_Print("Commands: f=all forward, b=all reverse; 1=FL F, 2=FL R, 3=FR F, 4=FR R, 5=RR F, 6=RR R, 7=RL F, 8=RL R; s=stop; status=read encoders.\r\n");
        UART_Print("Each motion test is 50% PWM for 10 seconds. Encoder telemetry is printed every 250 ms.\r\n");
        return;
    }

    if (std::strcmp(command, "status") == 0) {
        PrintTelemetry("STATUS | ");
        return;
    }

    UART_Print("Unknown command. Type help.\r\n");
}

static void ProcessSerial(void)
{
    // Drain a few bytes without blocking the motor timeout or telemetry scheduler.
    for (uint8_t n = 0U; n < 16U; ++n) {
        uint8_t received = 0U;
        if (HAL_UART_Receive(&huart2, &received, 1U, 0U) != HAL_OK) return;

        const char c = static_cast<char>(received);
        if (c == '\r' || c == '\n') {
            if (commandLength > 0U) {
                commandBuffer[commandLength] = '\0';
                ProcessCommand(commandBuffer);
                commandLength = 0U;
                std::memset(commandBuffer, 0, sizeof(commandBuffer));
            }
            continue;
        }

        if (commandLength < (sizeof(commandBuffer) - 1U)) {
            commandBuffer[commandLength++] = c;
        } else {
            commandLength = 0U;
            std::memset(commandBuffer, 0, sizeof(commandBuffer));
            UART_Print("Input too long; command discarded.\r\n");
        }
    }
}

int main(void)
{
    HAL_Init();
    SystemClock_Config();

    GPIO_Init();
    USART2_Init();
    TIM1_PWM_Init();
    TIM8_PWM_Init();
    Encoder_Init_All();

    StopAllMotors();
    if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1) != HAL_OK ||
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2) != HAL_OK ||
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3) != HAL_OK ||
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4) != HAL_OK ||
        HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1) != HAL_OK ||
        HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2) != HAL_OK ||
        HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3) != HAL_OK ||
        HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4) != HAL_OK) {
        Error_Handler();
    }
    // TIM1 and TIM8 are advanced-control timers; explicitly enable their main outputs.
    __HAL_TIM_MOE_ENABLE(&htim1);
    __HAL_TIM_MOE_ENABLE(&htim8);
    StopAllMotors();

    UART_Print("\r\n=== FOUR DRIVE MOTOR + ENCODER TEST ===\r\n");
    UART_Print("PWM: 50%, 20 kHz. Each motion test runs for 10 seconds, then stops.\r\n");
    UART_Print("FL: PA8/PA9 TIM1 CH1/CH2 | encoder PA0/PA1 TIM5 CH1/CH2.\r\n");
    UART_Print("FR: PA10/PA11 TIM1 CH3/CH4 (direction swapped) | encoder PB4/PB5 TIM3 CH1/CH2.\r\n");
    UART_Print("RR: PC8/PC9 TIM8 CH3/CH4 (direction swapped) | encoder PA15/PB3 TIM2 CH1/CH2.\r\n");
    UART_Print("RL: PC6/PC7 TIM8 CH1/CH2 | encoder PB6/PB7 TIM4 CH1/CH2.\r\n");
    UART_Print("Encoder mode: hardware TI12 quadrature x4.\r\n");
    UART_Print("Scale: 608972 counts/rev, 200 mm wheel; PROVISIONAL until measured.\r\n");
    UART_Print("Commands: f/b=all forward/reverse; 1..8=individual wheel/direction; s=stop; help.\r\n");
    UART_Print("Telemetry every 250 ms: signed raw counts, rotations and distance for all four wheels.\r\n");
    UART_Print("Direction correction: FR and RR forward/reverse outputs are swapped in software.\r\n");
    UART_Print("Steering PWM pins and steering sensors are not configured. Ready.\r\n");

    while (1) {
        Encoder_Update_All();
        ProcessSerial();

        const uint32_t now = HAL_GetTick();
        if (testActive) {
            if ((now - testStartMs) >= TEST_DURATION_MS) {
                StopAllMotors();
                testActive = false;
                PrintFinalSummary();
            } else if ((now - lastTelemetryMs) >= TELEMETRY_INTERVAL_MS) {
                lastTelemetryMs = now;
                char prefix[32] = {0};
                const uint32_t elapsed = now - testStartMs;
                std::snprintf(prefix, sizeof(prefix), "t=%lu.%01lu s | ",
                              static_cast<unsigned long>(elapsed / 1000U),
                              static_cast<unsigned long>((elapsed % 1000U) / 100U));
                PrintTelemetry(prefix);
            }
        }
    }
}
