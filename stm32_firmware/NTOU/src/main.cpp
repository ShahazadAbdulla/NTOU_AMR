#include "stm32f4xx_hal.h"
#include "drive.hpp"
#include "steering.hpp"

#include <cctype>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace {

UART_HandleTypeDef g_uart2 = {};
constexpr uint16_t kUartRxBufferSize = 128U;
constexpr uint16_t kUartRxBufferMask = kUartRxBufferSize - 1U;

uint8_t g_uartRxByte = 0U;

volatile uint8_t g_uartRxBuffer[kUartRxBufferSize]{};
volatile uint16_t g_uartRxHead = 0U;
volatile uint16_t g_uartRxTail = 0U;

volatile uint32_t g_uartRxOverflowCount = 0U;
volatile uint32_t g_uartRxRearmErrorCount = 0U;
volatile uint32_t g_uartRxErrorCount = 0U;

bool g_driveReady = false;
bool g_steeringReady = false;
char g_commandBuffer[96] = {};
uint8_t g_commandLength = 0U;
bool g_commandOverflow = false;
uint32_t g_lastTelemetryMs = 0U;

void Error_Handler()
{
    __disable_irq();
    while (1) { }
}

void SystemClock_Config()
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

void Usart2_Init()
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

    g_uart2.Instance = USART2;
    g_uart2.Init.BaudRate = 115200U;
    g_uart2.Init.WordLength = UART_WORDLENGTH_8B;
    g_uart2.Init.StopBits = UART_STOPBITS_1;
    g_uart2.Init.Parity = UART_PARITY_NONE;
    g_uart2.Init.Mode = UART_MODE_TX_RX;
    g_uart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    g_uart2.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&g_uart2) != HAL_OK) Error_Handler();
    HAL_NVIC_SetPriority(USART2_IRQn, 5U, 0U);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
}

void SerialPrint(const char *text)
{
    if (text == nullptr) return;
    HAL_UART_Transmit(&g_uart2,
                      reinterpret_cast<uint8_t *>(const_cast<char *>(text)),
                      static_cast<uint16_t>(std::strlen(text)),
                      200U);
}

void FormatInt64(int64_t value, char *out, size_t outSize)
{
    if (out == nullptr || outSize == 0U) return;
    char reversed[24];
    size_t count = 0U;
    const bool negative = value < 0;
    uint64_t magnitude = negative
        ? static_cast<uint64_t>(-(value + 1LL)) + 1ULL
        : static_cast<uint64_t>(value);
    do {
        reversed[count++] = static_cast<char>('0' + (magnitude % 10ULL));
        magnitude /= 10ULL;
    } while (magnitude != 0ULL && count < sizeof(reversed));

    size_t outIndex = 0U;
    if (negative && outIndex + 1U < outSize) out[outIndex++] = '-';
    while (count > 0U && outIndex + 1U < outSize) out[outIndex++] = reversed[--count];
    out[outIndex] = '\0';
}

void FormatFixed3(int64_t milliValue, char *out, size_t outSize)
{
    if (out == nullptr || outSize == 0U) return;
    const bool negative = milliValue < 0;
    const uint64_t magnitude = negative
        ? static_cast<uint64_t>(-(milliValue + 1LL)) + 1ULL
        : static_cast<uint64_t>(milliValue);
    char whole[24];
    FormatInt64(static_cast<int64_t>(magnitude / 1000ULL), whole, sizeof(whole));
    std::snprintf(out, outSize, "%s%s.%03lu",
                  negative ? "-" : "", whole,
                  static_cast<unsigned long>(magnitude % 1000ULL));
}

void FormatCdeg(int32_t cdeg, char *out, size_t outSize)
{
    const bool negative = cdeg < 0;
    const uint32_t magnitude = static_cast<uint32_t>(negative ? -cdeg : cdeg);
    std::snprintf(out, outSize, "%s%lu.%02lu",
                  negative ? "-" : "+",
                  static_cast<unsigned long>(magnitude / 100U),
                  static_cast<unsigned long>(magnitude % 100U));
}

const char *WheelName(uint8_t index)
{
    static const char *names[4] = {"FL", "FR", "RR", "RL"};
    return index < 4U ? names[index] : "?";
}

void PrintDriveStatus()
{
    for (uint8_t i = 0U; i < 4U; ++i) {
        Drive::EncoderSnapshot snapshot = {};
        const Drive::Wheel wheel = static_cast<Drive::Wheel>(i);
        if (!Drive::GetEncoderSnapshot(wheel, &snapshot)) continue;

        char raw[24], forward[24], rotations[32], distance[32], line[180];
        FormatInt64(snapshot.rawCounts, raw, sizeof(raw));
        FormatInt64(snapshot.forwardCounts, forward, sizeof(forward));
        FormatFixed3(snapshot.rotationsMilli, rotations, sizeof(rotations));
        FormatFixed3(snapshot.distanceMilliMm, distance, sizeof(distance));
        std::snprintf(line, sizeof(line),
            "%s pwm=%d%% raw=%s forward=%s rot=%s rev dist=%s mm\r\n",
            WheelName(i), static_cast<int>(snapshot.commandPercent),
            raw, forward, rotations, distance);
        SerialPrint(line);
    }
}

void PrintSteeringAxis(const char *name, Steering::Axis axis)
{
    Steering::AxisSnapshot snapshot = {};
    if (!Steering::GetAxisSnapshot(axis, &snapshot)) {
        SerialPrint(name);
        SerialPrint(" not initialized\r\n");
        return;
    }

    char angle[20], position[20], error[20], line[220];
    FormatCdeg(snapshot.angleCdeg, angle, sizeof(angle));
    FormatCdeg(snapshot.positionCdeg, position, sizeof(position));
    FormatCdeg(snapshot.errorCdeg, error, sizeof(error));
    const char *direction = snapshot.motorDirection > 0 ? "RIGHT" :
                            snapshot.motorDirection < 0 ? "LEFT" : "STOP";
    const char *sensorState = snapshot.sensorValid ? "OK" :
                              snapshot.holdover ? "HOLDOVER" : "INVALID";
    std::snprintf(line, sizeof(line),
        "%s raw=%u angle=%s pos=%s err=%s pwm=%u dir=%s status=0x%02X mag=%u sensor=%s\r\n",
        name, static_cast<unsigned>(snapshot.raw), angle, position, error,
        static_cast<unsigned>(snapshot.pwmPercent), direction,
        static_cast<unsigned>(snapshot.status), static_cast<unsigned>(snapshot.magnitude),
        sensorState);
    SerialPrint(line);
}

void PrintStatus()
{
    SerialPrint("\r\n--- NTOU STATUS ---\r\n");
    PrintDriveStatus();
    if (g_steeringReady) {
        PrintSteeringAxis("SR CH0", Steering::Axis::SR);
        PrintSteeringAxis("SL CH1", Steering::Axis::SL);
        char line[96];
        std::snprintf(line, sizeof(line), "steer_target=%d deg hold=%s tca=%s\r\n",
                      static_cast<int>(Steering::GetTargetDegrees()),
                      Steering::IsHolding() ? "ON" : "OFF",
                      Steering::IsTcaDetected() ? "DETECTED" : "MISSING");
        SerialPrint(line);
    } else {
        SerialPrint("Steering subsystem not initialized.\r\n");
    }
    SerialPrint("-------------------\r\n");
}

void PrintHelp()
{
    SerialPrint("NTOU controller commands (115200 baud, newline terminated):\r\n");
    SerialPrint("DRIVE <FL> <FR> <RR> <RL>  signed PWM percent, -100..100; +forward, -reverse\r\n");
    SerialPrint("F = all wheels +50%; B = all wheels -50%; DSTOP = drive stop\r\n");
    SerialPrint("STEER <degrees>            absolute target -40..+40; +RIGHT, -LEFT\r\n");
    SerialPrint("CENTER = STEER 0; SSTOP = disable steering hold; STOP = stop all motors\r\n");
    SerialPrint("STATUS = print encoders and steering feedback; HELP = this message\r\n");
    SerialPrint("Examples: DRIVE 40 40 40 40 | DRIVE 30 -30 30 -30 | STEER -15\r\n");
}

char *SkipSpaces(char *p)
{
    while (*p != '\0' && std::isspace(static_cast<unsigned char>(*p))) ++p;
    return p;
}

void Uppercase(char *text)
{
    for (; *text != '\0'; ++text) {
        *text = static_cast<char>(std::toupper(static_cast<unsigned char>(*text)));
    }
}

void ProcessCommand(char *command)
{
	SerialPrint("RXCMD=[");
	SerialPrint(command);
	SerialPrint("]\r\n");
	
    if (command == nullptr) return;
    command = SkipSpaces(command);
    size_t length = std::strlen(command);
    while (length > 0U && std::isspace(static_cast<unsigned char>(command[length - 1U]))) {
        command[--length] = '\0';
    }
    if (length == 0U) return;
    Uppercase(command);

    if (std::strcmp(command, "HELP") == 0 || std::strcmp(command, "?") == 0) {
        PrintHelp();
        return;
    }
    if (std::strcmp(command, "STATUS") == 0) {
        PrintStatus();
        return;
    }
    if (std::strcmp(command, "F") == 0 || std::strcmp(command, "FORWARD") == 0) {
        if (g_driveReady) Drive::SetAllWheelSpeeds(50, 50, 50, 50);
        SerialPrint("All drive wheels commanded forward at 50%.\r\n");
        return;
    }
    if (std::strcmp(command, "B") == 0 || std::strcmp(command, "BACKWARD") == 0 ||
        std::strcmp(command, "REVERSE") == 0) {
        if (g_driveReady) Drive::SetAllWheelSpeeds(-50, -50, -50, -50);
        SerialPrint("All drive wheels commanded reverse at 50%.\r\n");
        return;
    }
    if (std::strcmp(command, "DSTOP") == 0) {
        if (g_driveReady) Drive::StopAll();
        SerialPrint("Drive motors stopped.\r\n");
        return;
    }
    if (std::strcmp(command, "SSTOP") == 0) {
        if (g_steeringReady) Steering::DisableHold();
        SerialPrint("Steering motors stopped; position hold disabled.\r\n");
        return;
    }
    if (std::strcmp(command, "STOP") == 0) {
        if (g_driveReady) Drive::StopAll();
        if (g_steeringReady) Steering::DisableHold();
        SerialPrint("ALL MOTORS STOPPED.\r\n");
        return;
    }
    if (std::strcmp(command, "CENTER") == 0) {
        if (g_steeringReady && Steering::SetTargetDegrees(0)) {
            SerialPrint("Steering target set to centre (0 deg).\r\n");
        } else {
            SerialPrint("Steering subsystem unavailable.\r\n");
        }
        return;
    }

    if (std::strncmp(command, "DRIVE", 5U) == 0 &&
        (command[5] == '\0' || std::isspace(static_cast<unsigned char>(command[5])))) {
        int fl = 0, fr = 0, rr = 0, rl = 0;
        char extra = '\0';
        const int parsed = std::sscanf(command + 5, "%d %d %d %d %c", &fl, &fr, &rr, &rl, &extra);
        if (parsed != 4 || fl < -100 || fl > 100 || fr < -100 || fr > 100 ||
            rr < -100 || rr > 100 || rl < -100 || rl > 100) {
            SerialPrint("Invalid DRIVE command. Use: DRIVE <FL> <FR> <RR> <RL>, each -100..100.\r\n");
            return;
        }
        if (!g_driveReady) {
            SerialPrint("Drive subsystem unavailable.\r\n");
            return;
        }
        Drive::SetAllWheelSpeeds(static_cast<int16_t>(fl), static_cast<int16_t>(fr),
                                 static_cast<int16_t>(rr), static_cast<int16_t>(rl));
        SerialPrint("Drive wheel PWM commands updated.\r\n");
        return;
    }

    if (std::strncmp(command, "STEER", 5U) == 0 &&
        (command[5] == '\0' || std::isspace(static_cast<unsigned char>(command[5])))) {
        if (!g_steeringReady) {
            SerialPrint("Steering subsystem unavailable.\r\n");
            return;
        }
        char *end = nullptr;
        const long degrees = std::strtol(command + 5, &end, 10);
        if (end == command + 5) {
            SerialPrint("Invalid STEER command. Use STEER -40..40.\r\n");
            return;
        }
        end = SkipSpaces(end);
        if (*end != '\0' || degrees < -40L || degrees > 40L) {
            SerialPrint("Steering target must be an integer from -40 to +40 degrees.\r\n");
            return;
        }
        if (Steering::SetTargetDegrees(static_cast<int16_t>(degrees))) {
            SerialPrint("Steering target accepted; position hold enabled.\r\n");
        }
        return;
    }

    SerialPrint("Unknown command. Type HELP.\r\n");
}

void ProcessSerial()
{
    while (g_uartRxTail != g_uartRxHead) {

        // Read one byte from the interrupt-driven circular buffer.
        const uint8_t received = g_uartRxBuffer[g_uartRxTail];

        // Advance the buffer tail.
        g_uartRxTail = static_cast<uint16_t>(
            (g_uartRxTail + 1U) & kUartRxBufferMask);

        // Detect command termination.
        if (received == '\r' || received == '\n') {

            if (g_commandLength > 0U && !g_commandOverflow) {
                g_commandBuffer[g_commandLength] = '\0';
                ProcessCommand(g_commandBuffer);
            }
            else if (g_commandOverflow) {
                SerialPrint("Command too long; discarded.\r\n");
            }

            g_commandLength = 0U;
            g_commandOverflow = false;
            continue;
        }

        // Ignore remaining characters if the command overflowed.
        if (g_commandOverflow) {
            continue;
        }

        // Append character to the command buffer.
        if (g_commandLength < (sizeof(g_commandBuffer) - 1U)) {
            g_commandBuffer[g_commandLength++] = static_cast<char>(received);
        }
        else {
            g_commandOverflow = true;
        }
    }
}

void PrintPeriodicTelemetry(uint32_t nowMs)
{
    if ((nowMs - g_lastTelemetryMs) < 500U) return;
    g_lastTelemetryMs = nowMs;
    PrintDriveStatus();
    if (g_steeringReady) {
        PrintSteeringAxis("SR CH0", Steering::Axis::SR);
        PrintSteeringAxis("SL CH1", Steering::Axis::SL);
    }
}

} // namespace

extern "C" void USART2_IRQHandler(void)
{
    HAL_UART_IRQHandler(&g_uart2);
}

extern "C" void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != USART2) return;

    const uint16_t nextHead =
        static_cast<uint16_t>((g_uartRxHead + 1U) & kUartRxBufferMask);

    if (nextHead != g_uartRxTail) {
        g_uartRxBuffer[g_uartRxHead] = g_uartRxByte;
        g_uartRxHead = nextHead;
    } else {
        ++g_uartRxOverflowCount;
    }

    // Immediately arm reception for the next byte.
    if (HAL_UART_Receive_IT(&g_uart2, &g_uartRxByte, 1U) != HAL_OK) {
        ++g_uartRxRearmErrorCount;
    }
}

extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != USART2) return;

    ++g_uartRxErrorCount;

    if (HAL_UART_Receive_IT(&g_uart2, &g_uartRxByte, 1U) != HAL_OK) {
        ++g_uartRxRearmErrorCount;
    }
}

extern "C" void SysTick_Handler(void)
{
    HAL_IncTick();
    HAL_SYSTICK_IRQHandler();
}

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    Usart2_Init();
    
    if (HAL_UART_Receive_IT(&g_uart2, &g_uartRxByte, 1U) != HAL_OK) {
        Error_Handler();
    }
    
    g_driveReady = Drive::Init();
    g_steeringReady = Steering::Init();

    SerialPrint("\r\n=== NTOU STM32F446RE MAIN CONTROLLER ===\r\n");
    SerialPrint("Drive PWM: FL PA8/PA9; FR PA10/PA11 (forward/reverse swapped); ");
    SerialPrint("RR PC8/PC9 (forward/reverse swapped); RL PC6/PC7.\r\n");
    SerialPrint("Drive encoders: FL TIM5 PA0/PA1; FR TIM3 PB4/PB5; RR TIM2 PA15/PB3; RL TIM4 PB6/PB7.\r\n");
    SerialPrint("Steering: SR CH0 centre=5.45 deg; SL CH1 centre=208.04 deg; both position signs +1.\r\n");
    SerialPrint("Steering PWM: SR RIGHT PA6 / LEFT PA7; SL RIGHT PB15 / LEFT PB14 (swapped).\r\n");
    SerialPrint("No motors move automatically at startup. Waiting for laptop serial commands.\r\n");
    SerialPrint("Drive encoder distance scale 608972 counts/rev and 200 mm wheel are PROVISIONAL.\r\n");

    if (g_driveReady) SerialPrint("Drive subsystem initialized.\r\n");
    else SerialPrint("ERROR: drive subsystem initialization failed; drive commands disabled.\r\n");
    if (g_steeringReady) {
        SerialPrint(Steering::IsTcaDetected()
            ? "Steering subsystem initialized; TCA9548A detected.\r\n"
            : "Steering PWM/I2C initialized; TCA9548A not detected yet (will retry).\r\n");
    } else {
        SerialPrint("ERROR: steering subsystem initialization failed; steering commands disabled.\r\n");
    }
    PrintHelp();

    g_lastTelemetryMs = HAL_GetTick();
    while (1) {
        ProcessSerial();
        Drive::Update();
        const uint32_t now = HAL_GetTick();
        if (g_steeringReady) Steering::Update(now);
        PrintPeriodicTelemetry(now);
        HAL_Delay(1U);
    }
}
