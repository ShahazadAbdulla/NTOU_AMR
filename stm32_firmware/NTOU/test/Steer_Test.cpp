#include "stm32f4xx_hal.h"

#include <cstdint>
#include <cstdlib>
#include <cstdio>
#include <cstring>
#include <cmath>

// STM32F446RE - dual steering position controller using two AS5600 sensors.
//
// Steering convention:
//   Target + angle = RIGHT
//   Target - angle = LEFT
//   0 deg = the two measured mechanical centre positions below.
//
// TCA9548A / AS5600:
//   CH0 = SR, centre = 72.42 deg
//   CH1 = SL, centre = 202.06 deg
//
// PWM mapping:
//   SR RIGHT: PA6 / TIM13_CH1; SR LEFT: PA7 / TIM14_CH1
//   SL RIGHT: PB15 / TIM12_CH2; SL LEFT: PB14 / TIM12_CH1
//   NOTE: SL direction is deliberately swapped from the earlier motor test
//         because the IBT-2 wiring was changed.
//
// Commands:
//   -40 .. +40 : absolute steering target relative to centre, then hold
//   0          : return to the calibrated centre and hold
//   status     : print raw sensor values, positions, errors and PWM
//   s          : stop both motors and disable position hold
//   help / ?   : show commands
//
// IMPORTANT:
//   SL sensor polarity is based on the earlier observation that physical LEFT
//   increased its AS5600 angle. SR polarity has not yet been confirmed. Set
//   SR_POSITION_SIGN to +1 or -1 after the manual check described in the
//   handoff. The firmware refuses motion while it remains 0.

// ------------------------------ Configuration ------------------------------

static constexpr uint32_t PWM_PERIOD = 799U; // 20 kHz at 16 MHz timer clock
static constexpr uint32_t CONTROL_INTERVAL_MS = 20U; // 50 Hz controller
static constexpr uint32_t TELEMETRY_INTERVAL_MS = 250U;
static constexpr uint32_t SENSOR_DROPOUT_GRACE_MS = 1500U; // brief bracket/magnet interruptions
static constexpr uint32_t I2C_TIMEOUT_MS = 30U;
static constexpr uint32_t TCA_SETTLE_MS = 2U;

static constexpr uint8_t TCA_ADDRESS = 0x70U;
static constexpr uint8_t AS5600_ADDRESS = 0x36U;
static constexpr uint8_t AS5600_STATUS_REG = 0x0BU;
static constexpr uint8_t AS5600_RAW_MSB_REG = 0x0CU;
static constexpr uint8_t AS5600_MAG_MSB_REG = 0x1BU;

static constexpr uint8_t STATUS_MD = 0x20U;
static constexpr uint8_t STATUS_ML = 0x10U;
static constexpr uint8_t STATUS_MH = 0x08U;

static constexpr uint8_t SR_TCA_CHANNEL = 0U;
static constexpr uint8_t SL_TCA_CHANNEL = 1U;
static constexpr int32_t SR_CENTER_CDEG = 545;   // CH0: 5.45 degrees
static constexpr int32_t SL_CENTER_CDEG = 20804; // CH1: 208.04 degrees

// Normalized position is positive RIGHT and negative LEFT.
// SL: known previous observation - physical LEFT increased sensor angle.
static constexpr int8_t SL_POSITION_SIGN = +1;
// SR: NOT YET VERIFIED. After manually turning SR LEFT from centre:
//   if CH0 angle increases, set this to -1;
//   if CH0 angle decreases, set this to +1.
static constexpr int8_t SR_POSITION_SIGN = +1;

static constexpr int32_t TARGET_LIMIT_CDEG = 4000; // +/-40 deg
static constexpr int32_t HOLD_DEADBAND_CDEG = 150; // +/-1.50 deg

// Conservative initial control values. Previous steering motor test used 30%.
static constexpr float KP_PERCENT_PER_DEG = 1.8f;
static constexpr float KD_PERCENT_PER_DEG_PER_SEC = 0.25f;
static constexpr uint8_t MIN_PWM_PERCENT = 22U;
static constexpr uint8_t MAX_PWM_PERCENT = 40U;

// ------------------------------- HAL handles --------------------------------

TIM_HandleTypeDef htim12;
TIM_HandleTypeDef htim13;
TIM_HandleTypeDef htim14;
UART_HandleTypeDef huart2;
I2C_HandleTypeDef hi2c1;

struct SensorState {
    uint8_t channel;
    int32_t centerCdeg;
    int8_t positionSign;
    uint16_t raw;
    int32_t angleCdeg;
    int32_t positionCdeg;
    int32_t errorCdeg;
    uint16_t magnitude;
    uint8_t status;
    bool valid;
    bool hasPrevious;
    int32_t previousPositionCdeg;
    int32_t previousRawAngleCdeg;
    uint32_t previousSampleMs;
    float velocityDegPerSec;
    int8_t motorDirection; // +1 right, -1 left, 0 stopped
    uint8_t pwmPercent;
    bool dropoutActive;
    uint32_t dropoutStartMs;
};

static SensorState sr = {
    SR_TCA_CHANNEL, SR_CENTER_CDEG, SR_POSITION_SIGN,
    0U, 0, 0, 0, 0U, 0U, false, false, 0, 0, 0U, 0.0f, 0, 0U, false, 0U
};
static SensorState sl = {
    SL_TCA_CHANNEL, SL_CENTER_CDEG, SL_POSITION_SIGN,
    0U, 0, 0, 0, 0U, 0U, false, false, 0, 0, 0U, 0.0f, 0, 0U, false, 0U
};

static bool tcaFound = false;
static bool holdEnabled = false; // Require an explicit numeric target after boot.
static int32_t targetCdeg = 0;
static char commandBuffer[24] = {0};
static uint8_t commandLength = 0U;
static bool commandOverflow = false;

// -------------------------------- Prototypes --------------------------------

static void Error_Handler(void);
static void SystemClock_Config(void);
static void GPIO_Init(void);
static void USART2_Init(void);
static void I2C1_Init(void);
static void TIM12_Init(void);
static void TIM13_Init(void);
static void TIM14_Init(void);
static void StopAllMotors(void);
static void ProcessSerial(void);
static void ProcessCommand(const char *command);
static bool ReadSensor(SensorState *sensor, uint32_t nowMs);
static bool HandleSensorRead(SensorState *sensor, bool readOk, uint32_t nowMs);
static void UpdateMotorController(SensorState *sensor);
static void PrintStatus(void);

extern "C" void SysTick_Handler(void)
{
    HAL_IncTick();
    HAL_SYSTICK_IRQHandler();
}

static void Error_Handler(void)
{
    __disable_irq();
    while (1) {
    }
}

static void UART_Print(const char *text)
{
    if (text == nullptr) return;
    HAL_UART_Transmit(&huart2,
                      reinterpret_cast<uint8_t *>(const_cast<char *>(text)),
                      static_cast<uint16_t>(std::strlen(text)),
                      500U);
}

static void FormatCdeg(int32_t valueCdeg, char *out, size_t outSize)
{
    const bool negative = valueCdeg < 0;
    const uint32_t magnitude = static_cast<uint32_t>(negative ? -valueCdeg : valueCdeg);
    std::snprintf(out, outSize, "%s%lu.%02lu",
                  negative ? "-" : "+",
                  static_cast<unsigned long>(magnitude / 100U),
                  static_cast<unsigned long>(magnitude % 100U));
}

static int32_t SignedAngleDifferenceCdeg(int32_t angleCdeg, int32_t referenceCdeg)
{
    int32_t difference = angleCdeg - referenceCdeg;
    while (difference > 18000) difference -= 36000;
    while (difference < -18000) difference += 36000;
    return difference;
}

static void SystemClock_Config(void)
{
    RCC_OscInitTypeDef osc = {};
    RCC_ClkInitTypeDef clock = {};

    osc.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    osc.HSIState = RCC_HSI_ON;
    osc.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    osc.PLL.PLLState = RCC_PLL_NONE;
    if (HAL_RCC_OscConfig(&osc) != HAL_OK) Error_Handler();

    clock.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                      RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    clock.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
    clock.AHBCLKDivider = RCC_SYSCLK_DIV1;
    clock.APB1CLKDivider = RCC_HCLK_DIV1;
    clock.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&clock, FLASH_LATENCY_0) != HAL_OK) Error_Handler();
}

static void GPIO_Init(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitTypeDef gpio = {};
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_PULLDOWN;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;

    // Force steering PWM pins low before configuring timer alternate functions.
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

    // SL direction is swapped: RIGHT=PB15/TIM12_CH2, LEFT=PB14/TIM12_CH1.
    gpio.Pin = GPIO_PIN_14 | GPIO_PIN_15;
    gpio.Alternate = GPIO_AF9_TIM12;
    HAL_GPIO_Init(GPIOB, &gpio);
}

static void USART2_Init(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_USART2_CLK_ENABLE();

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
    if (HAL_UART_Init(&huart2) != HAL_OK) Error_Handler();
}

static void I2C1_Init(void)
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

    hi2c1.Instance = I2C1;
    hi2c1.Init.ClockSpeed = 100000U;
    hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1 = 0U;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2 = 0U;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c1) != HAL_OK) Error_Handler();
}

static void ConfigurePwmTimer(TIM_HandleTypeDef *timer, TIM_TypeDef *instance)
{
    timer->Instance = instance;
    timer->Init.Prescaler = 0U;
    timer->Init.CounterMode = TIM_COUNTERMODE_UP;
    timer->Init.Period = PWM_PERIOD;
    timer->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    timer->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_PWM_Init(timer) != HAL_OK) Error_Handler();

    TIM_OC_InitTypeDef channel = {};
    channel.OCMode = TIM_OCMODE_PWM1;
    channel.Pulse = 0U;
    channel.OCPolarity = TIM_OCPOLARITY_HIGH;
    channel.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(timer, &channel, TIM_CHANNEL_1) != HAL_OK)
        Error_Handler();
}

static void TIM12_Init(void)
{
    __HAL_RCC_TIM12_CLK_ENABLE();
    ConfigurePwmTimer(&htim12, TIM12);

    TIM_OC_InitTypeDef channel = {};
    channel.OCMode = TIM_OCMODE_PWM1;
    channel.Pulse = 0U;
    channel.OCPolarity = TIM_OCPOLARITY_HIGH;
    channel.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim12, &channel, TIM_CHANNEL_2) != HAL_OK)
        Error_Handler();
}

static void TIM13_Init(void)
{
    __HAL_RCC_TIM13_CLK_ENABLE();
    ConfigurePwmTimer(&htim13, TIM13);
}

static void TIM14_Init(void)
{
    __HAL_RCC_TIM14_CLK_ENABLE();
    ConfigurePwmTimer(&htim14, TIM14);
}

static bool TCA_SelectChannel(uint8_t channel)
{
    if (!tcaFound || channel > 7U) return false;
    uint8_t mask = static_cast<uint8_t>(1U << channel);
    if (HAL_I2C_Master_Transmit(&hi2c1,
                                static_cast<uint16_t>(TCA_ADDRESS << 1U),
                                &mask, 1U, I2C_TIMEOUT_MS) != HAL_OK) {
        return false;
    }
    HAL_Delay(TCA_SETTLE_MS);
    return true;
}

static bool TCA_Find(void)
{
    if (HAL_I2C_IsDeviceReady(&hi2c1,
                              static_cast<uint16_t>(TCA_ADDRESS << 1U),
                              2U, I2C_TIMEOUT_MS) == HAL_OK) {
        tcaFound = true;
        UART_Print("TCA9548A detected at 0x70.\r\n");
        return true;
    }
    tcaFound = false;
    UART_Print("ERROR: TCA9548A not responding at 0x70.\r\n");
    return false;
}

static bool ReadSensor(SensorState *sensor, uint32_t nowMs)
{
    if (sensor == nullptr) return false;
    sensor->valid = false;
    if (!TCA_SelectChannel(sensor->channel)) return false;

    uint8_t status = 0U;
    uint8_t rawBytes[2] = {0U, 0U};
    uint8_t magnitudeBytes[2] = {0U, 0U};

    if (HAL_I2C_Mem_Read(&hi2c1,
                         static_cast<uint16_t>(AS5600_ADDRESS << 1U),
                         AS5600_STATUS_REG, I2C_MEMADD_SIZE_8BIT,
                         &status, 1U, I2C_TIMEOUT_MS) != HAL_OK) return false;

    if (HAL_I2C_Mem_Read(&hi2c1,
                         static_cast<uint16_t>(AS5600_ADDRESS << 1U),
                         AS5600_RAW_MSB_REG, I2C_MEMADD_SIZE_8BIT,
                         rawBytes, 2U, I2C_TIMEOUT_MS) != HAL_OK) return false;

    if (HAL_I2C_Mem_Read(&hi2c1,
                         static_cast<uint16_t>(AS5600_ADDRESS << 1U),
                         AS5600_MAG_MSB_REG, I2C_MEMADD_SIZE_8BIT,
                         magnitudeBytes, 2U, I2C_TIMEOUT_MS) != HAL_OK) return false;

    sensor->status = status;
    sensor->raw = static_cast<uint16_t>(
        ((static_cast<uint16_t>(rawBytes[0]) & 0x0FU) << 8U) |
        static_cast<uint16_t>(rawBytes[1]));
    sensor->angleCdeg = static_cast<int32_t>(
        (static_cast<uint32_t>(sensor->raw) * 36000U + 2048U) / 4096U);
    sensor->magnitude = static_cast<uint16_t>(
        ((static_cast<uint16_t>(magnitudeBytes[0]) & 0x0FU) << 8U) |
        static_cast<uint16_t>(magnitudeBytes[1]));

    // Never run closed-loop steering with a missing, weak or overly strong magnet.
    const bool magnetDetected = (status & STATUS_MD) != 0U;
    const bool magnetWeak = (status & STATUS_ML) != 0U;
    const bool magnetStrong = (status & STATUS_MH) != 0U;
    if (!magnetDetected || magnetWeak || magnetStrong || sensor->magnitude < 10U) {
        sensor->valid = false;
        return false;
    }

    const int32_t rawPosition = SignedAngleDifferenceCdeg(sensor->angleCdeg,
                                                           sensor->centerCdeg);
    if (sensor->positionSign == 0) {
        // Still permit raw angle telemetry while SR polarity awaits confirmation.
        sensor->positionCdeg = 0;
    } else {
        sensor->positionCdeg = rawPosition * sensor->positionSign;
    }
    sensor->errorCdeg = targetCdeg - sensor->positionCdeg;

    if (sensor->hasPrevious && sensor->positionSign != 0) {
        const uint32_t elapsedMs = nowMs - sensor->previousSampleMs;
        if (elapsedMs > 0U) {
            int32_t deltaCdeg = SignedAngleDifferenceCdeg(sensor->angleCdeg,
                                                           sensor->previousRawAngleCdeg);
            deltaCdeg *= sensor->positionSign;
            sensor->velocityDegPerSec =
                static_cast<float>(deltaCdeg) / (100.0f * (static_cast<float>(elapsedMs) / 1000.0f));
        }
    } else {
        sensor->velocityDegPerSec = 0.0f;
    }

    sensor->previousPositionCdeg = sensor->positionCdeg;
    sensor->previousRawAngleCdeg = sensor->angleCdeg;
    sensor->previousSampleMs = nowMs;
    sensor->hasPrevious = true;
    sensor->valid = true;
    return true;
}

static bool HandleSensorRead(SensorState *sensor, bool readOk, uint32_t nowMs)
{
    if (sensor == nullptr) return false;

    const char *name = (sensor == &sr) ? "SR CH0" : "SL CH1";

    if (readOk) {
        if (sensor->dropoutActive) {
            sensor->dropoutActive = false;
            sensor->dropoutStartMs = 0U;
            // Do not calculate velocity across the missing-feedback interval.
            sensor->hasPrevious = false;
            sensor->velocityDegPerSec = 0.0f;
            char line[96];
            std::snprintf(line, sizeof(line), "%s sensor recovered; closed-loop control resumed.\r\n", name);
            UART_Print(line);
        }
        return true;
    }

    // When hold is disabled, no motor is running; do not start a grace timer.
    if (!holdEnabled) return true;

    if (!sensor->dropoutActive) {
        sensor->dropoutActive = true;
        sensor->dropoutStartMs = nowMs;
        char line[144];
        std::snprintf(line, sizeof(line),
                      "%s transient dropout: retaining its last motor direction/PWM for up to 1.5 s.\r\n",
                      name);
        UART_Print(line);
    }

    return (nowMs - sensor->dropoutStartMs) <= SENSOR_DROPOUT_GRACE_MS;
}

static void StopSR(void)
{
    __HAL_TIM_SET_COMPARE(&htim13, TIM_CHANNEL_1, 0U); // PA6, SR right
    __HAL_TIM_SET_COMPARE(&htim14, TIM_CHANNEL_1, 0U); // PA7, SR left
    sr.motorDirection = 0;
    sr.pwmPercent = 0U;
}

static void StopSL(void)
{
    __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, 0U); // PB15, SL right
    __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, 0U); // PB14, SL left
    sl.motorDirection = 0;
    sl.pwmPercent = 0U;
}

static void StopAllMotors(void)
{
    StopSR();
    StopSL();
}

static uint32_t DutyCompare(uint8_t percent)
{
    return ((PWM_PERIOD + 1U) * static_cast<uint32_t>(percent)) / 100U;
}

static void SetSRMotor(int8_t direction, uint8_t pwmPercent)
{
    __HAL_TIM_SET_COMPARE(&htim13, TIM_CHANNEL_1, 0U);
    __HAL_TIM_SET_COMPARE(&htim14, TIM_CHANNEL_1, 0U);
    if (direction > 0) {
        __HAL_TIM_SET_COMPARE(&htim13, TIM_CHANNEL_1, DutyCompare(pwmPercent)); // RIGHT PA6
    } else if (direction < 0) {
        __HAL_TIM_SET_COMPARE(&htim14, TIM_CHANNEL_1, DutyCompare(pwmPercent)); // LEFT PA7
    }
    sr.motorDirection = direction;
    sr.pwmPercent = (direction == 0) ? 0U : pwmPercent;
}

static void SetSLMotor(int8_t direction, uint8_t pwmPercent)
{
    __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, 0U); // PB15
    __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, 0U); // PB14
    if (direction > 0) {
        // SL direction reversed after IBT-2 replacement: RIGHT is PB15/CH2.
        __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, DutyCompare(pwmPercent));
    } else if (direction < 0) {
        // LEFT is PB14/CH1.
        __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, DutyCompare(pwmPercent));
    }
    sl.motorDirection = direction;
    sl.pwmPercent = (direction == 0) ? 0U : pwmPercent;
}

static uint8_t CalculateDuty(int32_t errorCdeg, float velocityDegPerSec)
{
    const float errorDeg = static_cast<float>(std::abs(errorCdeg)) / 100.0f;
    const float directionToTarget = (errorCdeg >= 0) ? 1.0f : -1.0f;
    const float approachVelocity = directionToTarget * velocityDegPerSec;
    const float damping = (approachVelocity > 0.0f)
        ? (KD_PERCENT_PER_DEG_PER_SEC * approachVelocity) : 0.0f;
    float duty = (KP_PERCENT_PER_DEG * errorDeg) - damping;

    if (duty < static_cast<float>(MIN_PWM_PERCENT)) duty = static_cast<float>(MIN_PWM_PERCENT);
    if (duty > static_cast<float>(MAX_PWM_PERCENT)) duty = static_cast<float>(MAX_PWM_PERCENT);
    return static_cast<uint8_t>(duty + 0.5f);
}

static void UpdateMotorController(SensorState *sensor)
{
    if (sensor == nullptr || !sensor->valid || !holdEnabled || sensor->positionSign == 0) {
        if (sensor == &sr) StopSR();
        else if (sensor == &sl) StopSL();
        return;
    }

    sensor->errorCdeg = targetCdeg - sensor->positionCdeg;
    if (std::abs(sensor->errorCdeg) <= HOLD_DEADBAND_CDEG) {
        if (sensor == &sr) StopSR();
        else StopSL();
        return;
    }

    const int8_t direction = (sensor->errorCdeg > 0) ? +1 : -1;
    const uint8_t duty = CalculateDuty(sensor->errorCdeg, sensor->velocityDegPerSec);
    if (sensor == &sr) SetSRMotor(direction, duty);
    else SetSLMotor(direction, duty);
}

static void PrintStatus(void)
{
    char srAngle[20], srPosition[20], srError[20];
    char slAngle[20], slPosition[20], slError[20], target[20];
    char line[400];
    const char *srHealth = sr.valid ? "OK" : (sr.dropoutActive ? "HOLDOVER" : "INVALID");
    const char *slHealth = sl.valid ? "OK" : (sl.dropoutActive ? "HOLDOVER" : "INVALID");

    FormatCdeg(sr.angleCdeg, srAngle, sizeof(srAngle));
    FormatCdeg(sr.positionCdeg, srPosition, sizeof(srPosition));
    FormatCdeg(sr.errorCdeg, srError, sizeof(srError));
    FormatCdeg(sl.angleCdeg, slAngle, sizeof(slAngle));
    FormatCdeg(sl.positionCdeg, slPosition, sizeof(slPosition));
    FormatCdeg(sl.errorCdeg, slError, sizeof(slError));
    FormatCdeg(targetCdeg, target, sizeof(target));

    std::snprintf(line, sizeof(line),
        "SR CH0 raw=%u angle=%s pos=%s err=%s pwm=%u dir=%s status=0x%02X mag=%u sensor=%s | "
        "SL CH1 raw=%u angle=%s pos=%s err=%s pwm=%u dir=%s status=0x%02X mag=%u sensor=%s | "
        "target=%s hold=%s\r\n",
        static_cast<unsigned>(sr.raw), srAngle,
        (SR_POSITION_SIGN == 0) ? "SIGN_UNSET" : srPosition,
        srError, static_cast<unsigned>(sr.pwmPercent),
        (sr.motorDirection > 0) ? "RIGHT" : (sr.motorDirection < 0) ? "LEFT" : "STOP",
        static_cast<unsigned>(sr.status), static_cast<unsigned>(sr.magnitude), srHealth,
        static_cast<unsigned>(sl.raw), slAngle, slPosition, slError,
        static_cast<unsigned>(sl.pwmPercent),
        (sl.motorDirection > 0) ? "RIGHT" : (sl.motorDirection < 0) ? "LEFT" : "STOP",
        static_cast<unsigned>(sl.status), static_cast<unsigned>(sl.magnitude), slHealth,
        target, holdEnabled ? "ON" : "OFF");
    UART_Print(line);
}

static void ProcessCommand(const char *command)
{
    if (command == nullptr || command[0] == '\0') return;

    if (std::strcmp(command, "help") == 0 || std::strcmp(command, "?") == 0) {
        UART_Print("Enter an integer target from -40 to +40 degrees; negative=LEFT, positive=RIGHT.\r\n");
        UART_Print("0=center; status=read values; s=stop and disable hold. Each target is held continuously.\r\n");
        UART_Print("SR sign is not configured yet. Verify SR sensor direction, then set SR_POSITION_SIGN to +1 or -1.\r\n");
        return;
    }

    if (std::strcmp(command, "status") == 0) {
        PrintStatus();
        return;
    }

    if (std::strcmp(command, "s") == 0 || std::strcmp(command, "stop") == 0) {
        holdEnabled = false;
        StopAllMotors();
        UART_Print("STOP: both steering motors disabled; position hold is OFF.\r\n");
        return;
    }

    if (std::strcmp(command, "center") == 0) {
        targetCdeg = 0;
    } else {
        char *end = nullptr;
        const long targetDegrees = std::strtol(command, &end, 10);
        if (end == command || *end != '\0' || targetDegrees < -40L || targetDegrees > 40L) {
            UART_Print("Invalid target. Enter an integer from -40 to +40, status, s, or help.\r\n");
            return;
        }
        targetCdeg = static_cast<int32_t>(targetDegrees * 100L);
    }

    if (SR_POSITION_SIGN != +1 && SR_POSITION_SIGN != -1) {
        UART_Print("MOTION BLOCKED: SR sensor polarity is unknown. Verify CH0 direction and set SR_POSITION_SIGN to +1 or -1.\r\n");
        holdEnabled = false;
        StopAllMotors();
        return;
    }

    holdEnabled = true;
    char targetText[20];
    char line[80];
    FormatCdeg(targetCdeg, targetText, sizeof(targetText));
    std::snprintf(line, sizeof(line), "Target set to %s degrees. Position hold enabled.\r\n", targetText);
    UART_Print(line);
}

static void ProcessSerial(void)
{
    uint8_t received = 0U;
    if (HAL_UART_Receive(&huart2, &received, 1U, 1U) != HAL_OK) return;

    if (received == '\r' || received == '\n') {
        if (commandLength > 0U && !commandOverflow) {
            commandBuffer[commandLength] = '\0';
            ProcessCommand(commandBuffer);
        } else if (commandOverflow) {
            UART_Print("Command too long; ignored.\r\n");
        }
        commandLength = 0U;
        commandOverflow = false;
        return;
    }

    if (commandOverflow) return;
    if (commandLength < (sizeof(commandBuffer) - 1U)) {
        commandBuffer[commandLength++] = static_cast<char>(received);
    } else {
        commandOverflow = true;
    }
}

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    GPIO_Init();
    USART2_Init();
    I2C1_Init();
    TIM12_Init();
    TIM13_Init();
    TIM14_Init();

    if (HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1) != HAL_OK ||
        HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2) != HAL_OK ||
        HAL_TIM_PWM_Start(&htim13, TIM_CHANNEL_1) != HAL_OK ||
        HAL_TIM_PWM_Start(&htim14, TIM_CHANNEL_1) != HAL_OK) {
        Error_Handler();
    }
    StopAllMotors();

    UART_Print("\r\n=== DUAL STEERING POSITION TEST ===\r\n");
    UART_Print("TCA CH0=SR centre 67.15 deg; CH1=SL centre 113.29 deg.\r\n");
    UART_Print("SR PWM: RIGHT PA6, LEFT PA7. SL PWM SWAPPED: RIGHT PB15, LEFT PB14.\r\n");
    UART_Print("Target convention: negative LEFT, positive RIGHT, range +/-40 deg.\r\n");
    UART_Print("PWM range 22-30%, 20 kHz; deadband +/-1.50 deg.\r\n");
    UART_Print("No motion at boot. Enter 0 to hold centre or a target such as -30 / +25.\r\n");
    UART_Print("Enter s to stop and disable hold; status to inspect both sensors.\r\n");
    if (SR_POSITION_SIGN == 0) {
        UART_Print("CONFIG REQUIRED: verify whether CH0 angle increases or decreases when SR is manually turned LEFT.\r\n");
        UART_Print("Then edit SR_POSITION_SIGN: angle increases => -1; angle decreases => +1.\r\n");
    }

    if (!TCA_Find()) {
        UART_Print("Steering outputs remain disabled until both sensors are valid.\r\n");
    }

    uint32_t lastControlMs = HAL_GetTick();
    uint32_t lastTelemetryMs = HAL_GetTick();
    uint32_t lastTcaRetryMs = HAL_GetTick();

    while (1) {
        ProcessSerial();
        const uint32_t now = HAL_GetTick();

        if (!tcaFound && (now - lastTcaRetryMs) >= 1000U) {
            lastTcaRetryMs = now;
            TCA_Find();
        }

        if ((now - lastControlMs) >= CONTROL_INTERVAL_MS) {
            lastControlMs = now;
            const bool srOk = ReadSensor(&sr, now);
            const bool slOk = ReadSensor(&sl, now);
            const bool srWithinGrace = HandleSensorRead(&sr, srOk, now);
            const bool slWithinGrace = HandleSensorRead(&sl, slOk, now);

            if (holdEnabled && (!srWithinGrace || !slWithinGrace)) {
                holdEnabled = false;
                StopAllMotors();
                UART_Print("SENSOR FAULT: AS5600 dropout exceeded 1.5 s. Both motors stopped; re-enter a target after sensor recovery.\r\n");
            } else if (holdEnabled) {
                // On a brief dropout, leave that motor's last PWM/direction untouched.
                // The other steering side continues updating from its valid sensor.
                if (srOk) UpdateMotorController(&sr);
                if (slOk) UpdateMotorController(&sl);
            } else {
                StopAllMotors();
            }
        }

        if ((now - lastTelemetryMs) >= TELEMETRY_INTERVAL_MS) {
            lastTelemetryMs = now;
            PrintStatus();
        }
    }
}
