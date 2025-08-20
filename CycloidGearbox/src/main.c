#include "main.h"

// ------- user-tunable knobs -------
#define TARGET_RPM 200  // desired speed
#define RUN_TIME_SEC 20 // run duration
#define CCW_DIRECTION 0 // 1 = CCW, 0 = CW
// ----------------------------------

// Forward declarations if you keep them elsewhere
void SystemClock_Config(void)
{
    // Your clock init here (left empty for brevity)
}

void Error_Handler(void)
{
    //__disable_irq();
    while (1)
    {
        __HAL_RCC_GPIOA_CLK_ENABLE();
        GPIO_InitTypeDef g = {0};
        g.Pin = GPIO_PIN_5;
        g.Mode = GPIO_MODE_OUTPUT_PP;
        g.Pull = GPIO_NOPULL;
        g.Speed = GPIO_SPEED_FREQ_LOW;
        HAL_GPIO_Init(GPIOA, &g);
        while (1)
        {
            HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
            HAL_Delay(100);
        }
    }
}

UART_HandleTypeDef huart2;
static void MX_USART2_UART_Init(void);     // <- prototype
int _write(int file, char *data, int len); // printf retarget (below)

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_USART2_UART_Init(); // 9600 right now
    setbuf(stdout, NULL);  // disable buffering for stdout
    HAL_Delay(1000);       // give the monitor a moment after reset
    printf("\r\nHELLO\r\n");
    const char *raw = "RAW\r\n";
    HAL_UART_Transmit(&huart2, (uint8_t *)raw, 5, 100);

    // Enable GPIO clocks BEFORE calling Motors_Init (Motor_Init uses GPIOB/GPIOA)
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    // Initialize your motor HAL (sets up GPIO and timers, incl. TIM3 IRQ for stepping)
    Motors_Init();

    // Compute signed angle from RPM and time
    // revolutions = rpm * (seconds / 60), angle = 2*pi*revs
    double revs = TARGET_RPM * (RUN_TIME_SEC / 60.0);
    double angle = revs * 2.0 * M_PI;
    if (!CCW_DIRECTION)
        angle = -angle;

    printf("\r\nCMD: rpm=%.1f, t=%.2fs -> revs=%.3f, angle=%.1f deg\r\n",
           TARGET_RPM, RUN_TIME_SEC, revs, (angle * 180.0 / M_PI));

    // Command the move. The function also needs a "speedRPM" argument
    // which you want to match TARGET_RPM for steady stepping.
    (void)MoveByAngle(&motor1, angle, TARGET_RPM);

    // Block until the motion completes (StepMotor() clears isMoving at the end)
    while (motor1.isMoving)
    {
        // you can sleep lightly to reduce CPU load
        HAL_Delay(1);
    }

    // Optionally stop timers defensively
    StopMotors();

    printf("DONE: motor stopped. Final revs=%.3f, angle=%.1f deg\r\n",
           revs, (angle * 180.0 / M_PI));

    // Idle
    while (1)
    {
        HAL_Delay(500);
    }
}

static void MX_USART2_UART_Init(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_USART2_CLK_ENABLE();

    GPIO_InitTypeDef g = {0};
    g.Pin = GPIO_PIN_2 | GPIO_PIN_3; // PA2=TX, PA3=RX
    g.Mode = GPIO_MODE_AF_PP;
    g.Pull = GPIO_PULLUP;
    g.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    g.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &g);

    huart2.Instance = USART2;
    huart2.Init.BaudRate = 9600;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart2) != HAL_OK)
    {
        Error_Handler();
    }
}

/* printf -> UART */
int _write(int file, char *data, int len)
{
    (void)file;
    HAL_UART_Transmit(&huart2, (uint8_t *)data, len, HAL_MAX_DELAY);
    return len;
}

void SysTick_Handler(void)
{
    HAL_IncTick(); // increments HAL tick for HAL_Delay / timeouts
    // HAL_SYSTICK_IRQHandler(); // optional, only if you use HAL's SysTick callbacks
}
