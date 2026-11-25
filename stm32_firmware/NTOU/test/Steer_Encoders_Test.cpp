#include "stm32f4xx_hal.h"
#include <cstdint>
#include <cstdio>
#include <cstring>

// STM32F446RE - manual test for the two steering AS5600 sensors.
// I2C1: PB8=SCL, PB9=SDA. TCA9548A address is discovered at 0x70..0x77.
// The firmware scans all eight mux channels, detects AS5600 devices at 0x36,
// then continuously prints their raw angle, degrees and magnet diagnostics.
// No motor PWM, motor driver, incremental encoder or steering control is used.
//
// Known project fact: SL AS5600 was previously verified on TCA9548A CH1.
// The second sensor channel is not assumed; identify it by rotating each wheel.

UART_HandleTypeDef huart2;
I2C_HandleTypeDef hi2c1;

static constexpr uint8_t TCA_ADDRESS_FIRST = 0x70U;
static constexpr uint8_t TCA_ADDRESS_LAST  = 0x77U;
static constexpr uint8_t AS5600_ADDRESS    = 0x36U;
static constexpr uint8_t AS5600_STATUS_REG = 0x0BU;
static constexpr uint8_t AS5600_RAW_MSB    = 0x0CU;
static constexpr uint8_t AS5600_AGC_REG    = 0x1AU;
static constexpr uint8_t AS5600_MAG_MSB    = 0x1BU;
static constexpr uint8_t STATUS_MD         = 0x20U;
static constexpr uint8_t STATUS_ML         = 0x10U;
static constexpr uint8_t STATUS_MH         = 0x08U;
static constexpr uint32_t SAMPLE_INTERVAL_MS = 250U;
static constexpr uint32_t I2C_TIMEOUT_MS = 50U;

static uint8_t tcaAddress = 0U;
static bool tcaFound = false;
static bool channelPresent[8] = {false, false, false, false, false, false, false, false};
static uint8_t sensorCount = 0U;

struct AS5600Sample {
    uint16_t raw;
    uint16_t angleCentiDegrees;
    uint16_t magnitude;
    uint8_t status;
    uint8_t agc;
};

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
                      static_cast<uint16_t>(strlen(text)),
                      1000U);
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

static bool TCA_FindAddress(void)
{
    for (uint8_t address = TCA_ADDRESS_FIRST; address <= TCA_ADDRESS_LAST; ++address) {
        if (HAL_I2C_IsDeviceReady(&hi2c1,
                                  static_cast<uint16_t>(address << 1U),
                                  2U,
                                  I2C_TIMEOUT_MS) == HAL_OK) {
            tcaAddress = address;
            tcaFound = true;
            char line[80];
            snprintf(line, sizeof(line), "TCA9548A detected at 0x%02X\r\n", tcaAddress);
            UART_Print(line);
            return true;
        }
    }

    tcaFound = false;
    UART_Print("ERROR: TCA9548A not detected at addresses 0x70-0x77.\r\n");
    return false;
}

static bool TCA_SelectChannel(uint8_t channel)
{
    if (!tcaFound || channel > 7U) return false;

    uint8_t channelMask = static_cast<uint8_t>(1U << channel);
    if (HAL_I2C_Master_Transmit(&hi2c1,
                                static_cast<uint16_t>(tcaAddress << 1U),
                                &channelMask,
                                1U,
                                I2C_TIMEOUT_MS) != HAL_OK) {
        return false;
    }

    // Allow the mux switch and downstream bus to settle before accessing AS5600.
    HAL_Delay(2U);
    return true;
}

static bool AS5600_ReadRegister(uint8_t reg, uint8_t *data, uint16_t length)
{
    return HAL_I2C_Mem_Read(&hi2c1,
                            static_cast<uint16_t>(AS5600_ADDRESS << 1U),
                            reg,
                            I2C_MEMADD_SIZE_8BIT,
                            data,
                            length,
                            I2C_TIMEOUT_MS) == HAL_OK;
}

static bool AS5600_ReadChannel(uint8_t channel, AS5600Sample *sample)
{
    if (sample == nullptr || !TCA_SelectChannel(channel)) return false;

    uint8_t status = 0U;
    uint8_t rawBytes[2] = {0U, 0U};
    uint8_t agc = 0U;
    uint8_t magnitudeBytes[2] = {0U, 0U};

    if (!AS5600_ReadRegister(AS5600_STATUS_REG, &status, 1U)) return false;
    if (!AS5600_ReadRegister(AS5600_RAW_MSB, rawBytes, 2U)) return false;
    if (!AS5600_ReadRegister(AS5600_AGC_REG, &agc, 1U)) return false;
    if (!AS5600_ReadRegister(AS5600_MAG_MSB, magnitudeBytes, 2U)) return false;

    const uint16_t raw = static_cast<uint16_t>(
        ((static_cast<uint16_t>(rawBytes[0]) & 0x0FU) << 8U) |
        static_cast<uint16_t>(rawBytes[1]));

    sample->raw = raw;
    sample->angleCentiDegrees = static_cast<uint16_t>(
        (static_cast<uint32_t>(raw) * 36000U + 2048U) / 4096U);
    sample->status = status;
    sample->agc = agc;
    sample->magnitude = static_cast<uint16_t>(
        ((static_cast<uint16_t>(magnitudeBytes[0]) & 0x0FU) << 8U) |
        static_cast<uint16_t>(magnitudeBytes[1]));
    return true;
}

static void PrintSample(uint8_t channel, const AS5600Sample &sample)
{
    char line[180];
    const char *label = (channel == 1U) ? "known-SL" : "identify-side";
    const bool magnet = (sample.status & STATUS_MD) != 0U;
    const bool weak = (sample.status & STATUS_ML) != 0U;
    const bool strong = (sample.status & STATUS_MH) != 0U;

    snprintf(line, sizeof(line),
             "CH%u [%s] RAW=%u ANGLE=%u.%02u deg STATUS=0x%02X "
             "MAGNET=%s WEAK=%s STRONG=%s AGC=%u MAGNITUDE=%u\r\n",
             static_cast<unsigned>(channel), label,
             static_cast<unsigned>(sample.raw),
             static_cast<unsigned>(sample.angleCentiDegrees / 100U),
             static_cast<unsigned>(sample.angleCentiDegrees % 100U),
             static_cast<unsigned>(sample.status),
             magnet ? "YES" : "NO",
             weak ? "YES" : "NO",
             strong ? "YES" : "NO",
             static_cast<unsigned>(sample.agc),
             static_cast<unsigned>(sample.magnitude));
    UART_Print(line);
}

static uint8_t ScanAllChannels(void)
{
    sensorCount = 0U;
    for (uint8_t channel = 0U; channel < 8U; ++channel) {
        channelPresent[channel] = false;
        if (!TCA_SelectChannel(channel)) {
            char line[64];
            snprintf(line, sizeof(line), "CH%u: mux selection failed\r\n",
                     static_cast<unsigned>(channel));
            UART_Print(line);
            continue;
        }

        if (HAL_I2C_IsDeviceReady(&hi2c1,
                                  static_cast<uint16_t>(AS5600_ADDRESS << 1U),
                                  2U,
                                  I2C_TIMEOUT_MS) != HAL_OK) {
            char line[48];
            snprintf(line, sizeof(line), "CH%u: no AS5600 at 0x36\r\n",
                     static_cast<unsigned>(channel));
            UART_Print(line);
            continue;
        }

        AS5600Sample sample = {};
        if (AS5600_ReadChannel(channel, &sample)) {
            channelPresent[channel] = true;
            ++sensorCount;
            PrintSample(channel, sample);
        } else {
            char line[64];
            snprintf(line, sizeof(line), "CH%u: AS5600 ACK, but register read failed\r\n",
                     static_cast<unsigned>(channel));
            UART_Print(line);
        }
    }

    char summary[96];
    snprintf(summary, sizeof(summary),
             "Scan complete: %u AS5600 sensor(s) detected across TCA channels.\r\n\r\n",
             static_cast<unsigned>(sensorCount));
    UART_Print(summary);
    return sensorCount;
}

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    USART2_Init();
    I2C1_Init();

    UART_Print("\r\n=== STEERING AS5600 ENCODER TEST ===\r\n");
    UART_Print("No motors or PWM are enabled in this firmware.\r\n");
    UART_Print("I2C1: PB8=SCL, PB9=SDA | AS5600 address: 0x36 | TCA9548A: 0x70-0x77\r\n");
    UART_Print("Scanning all eight TCA channels. CH1 is the previously verified SL sensor.\r\n\r\n");

    while (!TCA_FindAddress()) {
        UART_Print("Retrying TCA9548A detection in 2 seconds...\r\n");
        HAL_Delay(2000U);
    }

    ScanAllChannels();
    UART_Print("Manual test: rotate the SL and SR wheels by hand; watch which channel's RAW/ANGLE changes.\r\n");
    UART_Print("Live readings update every 250 ms. Press Ctrl+C to close the monitor.\r\n\r\n");

    uint32_t lastSampleMs = HAL_GetTick();
    uint32_t lastRescanMs = HAL_GetTick();

    while (1) {
        const uint32_t now = HAL_GetTick();

        if ((sensorCount == 0U) && ((now - lastRescanMs) >= 2000U)) {
            lastRescanMs = now;
            UART_Print("No sensors found on the last scan; rescanning all TCA channels...\r\n");
            if (!tcaFound) TCA_FindAddress();
            if (tcaFound) ScanAllChannels();
        }

        if ((now - lastSampleMs) >= SAMPLE_INTERVAL_MS) {
            lastSampleMs = now;
            for (uint8_t channel = 0U; channel < 8U; ++channel) {
                if (!channelPresent[channel]) continue;

                AS5600Sample sample = {};
                if (AS5600_ReadChannel(channel, &sample)) {
                    PrintSample(channel, sample);
                } else {
                    char line[64];
                    snprintf(line, sizeof(line), "CH%u READ_ERROR (checking next sample)\r\n",
                             static_cast<unsigned>(channel));
                    UART_Print(line);
                }
            }
        }
    }
}
