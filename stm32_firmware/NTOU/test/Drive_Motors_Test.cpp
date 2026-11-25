#include "stm32f4xx_hal.h"
#include <cstring>
#include <cstdint>

// STM32F446RE - four drive motors only.
// Steering outputs (PB14/PB15 and PA6/PA7) are deliberately not configured.
// USART2 PA2/PA3 is used for the Nucleo ST-LINK virtual COM port.

UART_HandleTypeDef huart2;
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim8;

static constexpr uint32_t PWM_PERIOD = 799U;      // 16 MHz / (799 + 1) = 20 kHz
static constexpr uint32_t PWM_DUTY_50 = 400U;      // 50% of 800 timer counts
static constexpr uint32_t TEST_DURATION_MS = 3000U;
static constexpr uint32_t UART_TIMEOUT_MS = 1U;

static bool testActive = false;
static uint32_t testStartMs = 0U;
static char commandBuffer[32] = {0};
static uint8_t commandLength = 0U;

static void Error_Handler(void);
static void GPIO_Init(void);
static void USART2_Init(void);
static void TIM1_Init(void);
static void TIM8_Init(void);
static void UART_Print(const char *text);
static void StopAllMotors(void);
static void SetMotor(uint8_t motor, bool forward);
static void StartTestCommand(char command);
static void ProcessCommand(const char *command);
static void ProcessSerial(void);

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

static void GPIO_Init(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    GPIO_InitTypeDef gpio = {};

    // Put all eight PWM pins at a low output before switching them to AF mode.
    gpio.Pin = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &gpio);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11, GPIO_PIN_RESET);

    gpio.Pin = GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9;
    HAL_GPIO_Init(GPIOC, &gpio);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9, GPIO_PIN_RESET);

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
    huart2.Init.BaudRate = 115200;
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

static void ConfigurePwmTimer(TIM_HandleTypeDef *htim, TIM_TypeDef *instance)
{
    htim->Instance = instance;
    htim->Init.Prescaler = 0U;
    htim->Init.CounterMode = TIM_COUNTERMODE_UP;
    htim->Init.Period = PWM_PERIOD;
    htim->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim->Init.RepetitionCounter = 0U;
    htim->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    if (HAL_TIM_PWM_Init(htim) != HAL_OK) {
        Error_Handler();
    }

    TIM_OC_InitTypeDef oc = {};
    oc.OCMode = TIM_OCMODE_PWM1;
    oc.Pulse = 0U;
    oc.OCPolarity = TIM_OCPOLARITY_HIGH;
    oc.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(htim, &oc, TIM_CHANNEL_1) != HAL_OK ||
        HAL_TIM_PWM_ConfigChannel(htim, &oc, TIM_CHANNEL_2) != HAL_OK ||
        HAL_TIM_PWM_ConfigChannel(htim, &oc, TIM_CHANNEL_3) != HAL_OK ||
        HAL_TIM_PWM_ConfigChannel(htim, &oc, TIM_CHANNEL_4) != HAL_OK) {
        Error_Handler();
    }
}

static void TIM1_Init(void)
{
    __HAL_RCC_TIM1_CLK_ENABLE();
    ConfigurePwmTimer(&htim1, TIM1);
}

static void TIM8_Init(void)
{
    __HAL_RCC_TIM8_CLK_ENABLE();
    ConfigurePwmTimer(&htim8, TIM8);
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
    // FL and FR use TIM1.
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0U);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0U);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0U);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0U);

    // RL and RR use TIM8.
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, 0U);
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, 0U);
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, 0U);
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, 0U);
}

// Motor index: 0=FL, 1=FR, 2=RR, 3=RL.
// forward=true drives RPWM; forward=false drives LPWM.
static void SetMotor(uint8_t motor, bool forward)
{
    TIM_HandleTypeDef *timer = nullptr;
    uint32_t forwardChannel = 0U;
    uint32_t reverseChannel = 0U;

    switch (motor) {
        case 0U: // FL: PA8 RPWM / PA9 LPWM, TIM1 CH1/CH2
            timer = &htim1;
            forwardChannel = TIM_CHANNEL_1;
            reverseChannel = TIM_CHANNEL_2;
            break;
        case 1U: // FR: PA10 RPWM / PA11 LPWM, TIM1 CH3/CH4
            timer = &htim1;
            forwardChannel = TIM_CHANNEL_4;
            reverseChannel = TIM_CHANNEL_3;
            break;
        case 2U: // RR: PC8 RPWM / PC9 LPWM, TIM8 CH3/CH4
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

    // Break-before-make: force both inputs low before enabling one direction.
    __HAL_TIM_SET_COMPARE(timer, forwardChannel, 0U);
    __HAL_TIM_SET_COMPARE(timer, reverseChannel, 0U);
    if (forward) {
        __HAL_TIM_SET_COMPARE(timer, forwardChannel, PWM_DUTY_50);
    } else {
        __HAL_TIM_SET_COMPARE(timer, reverseChannel, PWM_DUTY_50);
    }
}

static void StartTestCommand(char command)
{
    StopAllMotors();

    switch (command) {
        case 'f':
            SetMotor(0U, true);  // FL forward
            SetMotor(1U, true);  // FR forward
            SetMotor(2U, true);  // RR forward
            SetMotor(3U, true);  // RL forward
            UART_Print("TEST: all four drive motors FORWARD at 50% for 1 second.\r\n");
            break;
        case 'b':
            SetMotor(0U, false); // FL reverse
            SetMotor(1U, false); // FR reverse
            SetMotor(2U, false); // RR reverse
            SetMotor(3U, false); // RL reverse
            UART_Print("TEST: all four drive motors REVERSE at 50% for 1 second.\r\n");
            break;
        case '1': SetMotor(0U, true);  UART_Print("TEST: FL FORWARD at 50% for 1 second.\r\n"); break;
        case '2': SetMotor(0U, false); UART_Print("TEST: FL REVERSE at 50% for 1 second.\r\n"); break;
        case '3': SetMotor(1U, true);  UART_Print("TEST: FR FORWARD at 50% for 1 second.\r\n"); break;
        case '4': SetMotor(1U, false); UART_Print("TEST: FR REVERSE at 50% for 1 second.\r\n"); break;
        case '5': SetMotor(2U, true);  UART_Print("TEST: RR FORWARD at 50% for 1 second.\r\n"); break;
        case '6': SetMotor(2U, false); UART_Print("TEST: RR REVERSE at 50% for 1 second.\r\n"); break;
        case '7': SetMotor(3U, true);  UART_Print("TEST: RL FORWARD at 50% for 1 second.\r\n"); break;
        case '8': SetMotor(3U, false); UART_Print("TEST: RL REVERSE at 50% for 1 second.\r\n"); break;
        default:
            return;
    }

    testStartMs = HAL_GetTick();
    testActive = true;
}

static void ProcessCommand(const char *command)
{
    if (command == nullptr || command[0] == '\0') {
        return;
    }

    if (std::strlen(command) == 1U) {
        const char c = command[0];
        if (c == 's' || c == 'S') {
            StopAllMotors();
            testActive = false;
            UART_Print("STOP: all four drive motors disabled.\r\n");
            return;
        }
        if (c == 'f' || c == 'F' || c == 'b' || c == 'B' ||
            (c >= '1' && c <= '8')) {
            const char normalized = (c >= 'A' && c <= 'Z') ? static_cast<char>(c - 'A' + 'a') : c;
            StartTestCommand(normalized);
            return;
        }
    }

    if (std::strcmp(command, "help") == 0 || std::strcmp(command, "?") == 0) {
        UART_Print("Commands: f=all forward, b=all reverse, 1=FL F, 2=FL R, 3=FR F, 4=FR R, 5=RR F, 6=RR R, 7=RL F, 8=RL R, s=stop. Each drive test is 1 second at 50%.\r\n");
        return;
    }

    UART_Print("Unknown command. Type help.\r\n");
}

static void ProcessSerial(void)
{
    uint8_t received = 0U;
    if (HAL_UART_Receive(&huart2, &received, 1U, UART_TIMEOUT_MS) != HAL_OK) {
        return;
    }

    const char c = static_cast<char>(received);
    if (c == '\r' || c == '\n') {
        if (commandLength > 0U) {
            commandBuffer[commandLength] = '\0';
            ProcessCommand(commandBuffer);
            commandLength = 0U;
            std::memset(commandBuffer, 0, sizeof(commandBuffer));
        }
        return;
    }

    if (commandLength < (sizeof(commandBuffer) - 1U)) {
        commandBuffer[commandLength++] = c;
    } else {
        commandLength = 0U;
        std::memset(commandBuffer, 0, sizeof(commandBuffer));
        UART_Print("Input too long; command discarded.\r\n");
    }
}

int main(void)
{
    HAL_Init();
    // The STM32F446RE remains on its reset-default 16 MHz HSI clock.
    // TIM1 and TIM8 therefore receive a 16 MHz timer clock with APB2 prescaler /1.

    GPIO_Init();
    USART2_Init();
    TIM1_Init();
    TIM8_Init();

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

    StopAllMotors();

    UART_Print("\r\n=== FOUR DRIVE MOTOR UNIT TEST ===\r\n");
    UART_Print("FL: PA8/PA9 TIM1 CH1/CH2 | FR: PA10/PA11 TIM1 CH3/CH4\r\n");
    UART_Print("RR: PC8/PC9 TIM8 CH3/CH4 | RL: PC6/PC7 TIM8 CH1/CH2\r\n");
    UART_Print("f=all forward, b=all reverse; 1..8 test each direction individually.\r\n");
    UART_Print("Every test runs at 50% PWM for 1 second, then stops. s=stop, help=commands.\r\n");
    UART_Print("Steering pins are not configured or used. Ready.\r\n");

    while (1) {
        ProcessSerial();
        if (testActive && ((HAL_GetTick() - testStartMs) >= TEST_DURATION_MS)) {
            StopAllMotors();
            testActive = false;
            UART_Print("TEST COMPLETE: all drive PWM outputs stopped.\r\n");
        }
    }
}
