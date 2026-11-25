#include "stm32f4xx_hal.h"

#include <cstdint>
#include <cstring>

// STM32F446RE — both steering motors, PWM-only unit test.
//
// Commands over USART2 / ST-LINK VCP at 115200 baud:
//   r, rr, rrr, rrrr, rrrrr : both steering motors RIGHT for 1..5 seconds
//   l, ll, lll, llll, lllll : both steering motors LEFT for 1..5 seconds
//   s                         : stop both motors immediately
//   help                      : show commands
//
// PWM frequency: 20 kHz. Duty: 30% (matches the previous SL motor test).
// No AS5600, TCA9548A, encoders, drive motors, or position controller.
//
// Board PWM mapping from the supplied schematic:
//   SL right: PB14 / TIM12_CH1; SL left: PB15 / TIM12_CH2
//   SR right: PA6  / TIM13_CH1; SR left: PA7  / TIM14_CH1

static constexpr uint32_t PWM_PERIOD = 799U;                 // 20 kHz at 16 MHz
static constexpr uint32_t PWM_DUTY_30_PERCENT = 240U;        // 30% of 800 counts
static constexpr uint32_t MAX_RUN_TIME_MS = 5000U;

TIM_HandleTypeDef htim12;
TIM_HandleTypeDef htim13;
TIM_HandleTypeDef htim14;
UART_HandleTypeDef huart2;

static bool motorsRunning = false;
static uint32_t motorStopAtMs = 0U;
static char commandBuffer[8] = {0};
static uint8_t commandLength = 0U;
static bool commandOverflow = false;

static void Error_Handler(void);
static void SystemClock_Config(void);
static void GPIO_Init(void);
static void USART2_Init(void);
static void ConfigurePwmTimer(TIM_HandleTypeDef *timer, TIM_TypeDef *instance);
static void TIM12_Init(void);
static void TIM13_Init(void);
static void TIM14_Init(void);
static void StopBothSteeringMotors(void);
static void RunBothSteeringMotors(bool right, uint8_t seconds);
static void UART_Print(const char *text);
static void ProcessCommand(const char *command);
static void ProcessSerial(void);

extern "C" void SysTick_Handler(void)
{
    HAL_IncTick();
    HAL_SYSTICK_IRQHandler();
}

static void Error_Handler(void)
{
    // Outputs are initialized low and PWM compare values start at zero.
    __disable_irq();
    while (1) {
    }
}

static void UART_Print(const char *text)
{
    if (text == nullptr) return;
    HAL_UART_Transmit(
        &huart2,
        reinterpret_cast<uint8_t *>(const_cast<char *>(text)),
        static_cast<uint16_t>(std::strlen(text)),
        200U
    );
}

static void SystemClock_Config(void)
{
    // HSI 16 MHz, APB1/APB2 undivided. TIM12/13/14 therefore run at 16 MHz.
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

    // Hold all steering PWM pins low as GPIO before configuring alternate function.
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_PULLDOWN;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    gpio.Pin = GPIO_PIN_6 | GPIO_PIN_7;
    HAL_GPIO_Init(GPIOA, &gpio);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6 | GPIO_PIN_7, GPIO_PIN_RESET);

    gpio.Pin = GPIO_PIN_14 | GPIO_PIN_15;
    HAL_GPIO_Init(GPIOB, &gpio);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14 | GPIO_PIN_15, GPIO_PIN_RESET);

    // SR: PA6=TIM13_CH1 and PA7=TIM14_CH1, AF9.
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_PULLDOWN;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = GPIO_AF9_TIM13;
    gpio.Pin = GPIO_PIN_6;
    HAL_GPIO_Init(GPIOA, &gpio);

    gpio.Alternate = GPIO_AF9_TIM14;
    gpio.Pin = GPIO_PIN_7;
    HAL_GPIO_Init(GPIOA, &gpio);

    // SL: PB14=TIM12_CH1 and PB15=TIM12_CH2, AF9.
    gpio.Alternate = GPIO_AF9_TIM12;
    gpio.Pin = GPIO_PIN_14 | GPIO_PIN_15;
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

    // TIM12 has two channels for SL.
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

static void StopBothSteeringMotors(void)
{
    // Set every steering H-bridge PWM input to zero.
    __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, 0U); // SL right
    __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, 0U); // SL left
    __HAL_TIM_SET_COMPARE(&htim13, TIM_CHANNEL_1, 0U); // SR right
    __HAL_TIM_SET_COMPARE(&htim14, TIM_CHANNEL_1, 0U); // SR left
    motorsRunning = false;
}

static void RunBothSteeringMotors(bool right, uint8_t seconds)
{
    StopBothSteeringMotors();

    if (right) {
        // Both motors receive their designated RIGHT PWM input.
        __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, PWM_DUTY_30_PERCENT); // PB14
        __HAL_TIM_SET_COMPARE(&htim13, TIM_CHANNEL_1, PWM_DUTY_30_PERCENT); // PA6
    } else {
        // Both motors receive their designated LEFT PWM input.
        __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, PWM_DUTY_30_PERCENT); // PB15
        __HAL_TIM_SET_COMPARE(&htim14, TIM_CHANNEL_1, PWM_DUTY_30_PERCENT); // PA7
    }

    motorStopAtMs = HAL_GetTick() + (static_cast<uint32_t>(seconds) * 1000U);
    motorsRunning = true;

    if (right) {
        UART_Print("RIGHT: SL PB14 + SR PA6 at 30% PWM for ");
    } else {
        UART_Print("LEFT: SL PB15 + SR PA7 at 30% PWM for ");
    }

    // Print duration without requiring printf integer formatting.
    const char durationText[2] = {static_cast<char>('0' + seconds), '\0'};
    UART_Print(durationText);
    UART_Print(" second(s).\r\n");
}

static void ProcessCommand(const char *command)
{
    if (command == nullptr || command[0] == '\0') return;

    if (std::strcmp(command, "s") == 0 || std::strcmp(command, "S") == 0) {
        StopBothSteeringMotors();
        UART_Print("STOP: both steering motors stopped.\r\n");
        return;
    }

    if (std::strcmp(command, "help") == 0 || std::strcmp(command, "?") == 0) {
        UART_Print("Commands: r..rrrrr = both RIGHT for 1..5 s; l..lllll = both LEFT for 1..5 s; s=stop.\r\n");
        UART_Print("PWM=30%, 20 kHz. Only steering motors are enabled.\r\n");
        return;
    }

    const char direction = command[0];
    if (direction != 'r' && direction != 'R' && direction != 'l' && direction != 'L') {
        UART_Print("Invalid command. Use r, rr, ... rrrrr; l, ll, ... lllll; or s.\r\n");
        return;
    }

    const size_t length = std::strlen(command);
    if (length < 1U || length > 5U) {
        UART_Print("Invalid duration. Use one to five repeated r or l characters.\r\n");
        return;
    }

    for (size_t i = 0U; i < length; ++i) {
        const char c = command[i];
        if (c != direction && !(direction == 'r' && c == 'R') && !(direction == 'l' && c == 'L')) {
            UART_Print("Invalid command. Use only repeated r or only repeated l.\r\n");
            return;
        }
    }

    RunBothSteeringMotors(direction == 'r' || direction == 'R', static_cast<uint8_t>(length));
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
    TIM12_Init();
    TIM13_Init();
    TIM14_Init();

    StopBothSteeringMotors();

    if (HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1) != HAL_OK ||
        HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2) != HAL_OK ||
        HAL_TIM_PWM_Start(&htim13, TIM_CHANNEL_1) != HAL_OK ||
        HAL_TIM_PWM_Start(&htim14, TIM_CHANNEL_1) != HAL_OK) {
        Error_Handler();
    }

    StopBothSteeringMotors();

    UART_Print("\r\n=== BOTH STEERING MOTORS PWM TEST ===\r\n");
    UART_Print("SL: RIGHT=PB14/TIM12_CH1, LEFT=PB15/TIM12_CH2.\r\n");
    UART_Print("SR: RIGHT=PA6/TIM13_CH1, LEFT=PA7/TIM14_CH1.\r\n");
    UART_Print("PWM=30%, 20 kHz. r..rrrrr=RIGHT 1..5 s; l..lllll=LEFT 1..5 s; s=stop.\r\n");
    UART_Print("No AS5600, TCA9548A, encoders, or drive motors are initialized.\r\n");
    UART_Print("Ready. Enter one command per line.\r\n");

    while (1) {
        ProcessSerial();

        if (motorsRunning && static_cast<int32_t>(HAL_GetTick() - motorStopAtMs) >= 0) {
            StopBothSteeringMotors();
            UART_Print("TIMEOUT: both steering motors stopped.\r\n");
        }
    }
}
