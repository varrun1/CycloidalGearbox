#include "main.h"

// ------- Input Data & Structs -------
#define TARGET_RPM 200   // desired speed
#define RUN_TIME_SEC 20  // run duration
#define CCW_DIRECTION 1  // 1 = CCW, 0 = CW
#define output_angle 3.0 // in degrees

const int cycles = 2;          // how many CW landings to sample
const double retreatDeg = 5.0; // back off amount between landings

LoadCell cell_1 = {
    .D_Port = GPIOA,
    .D_Pin = GPIO_PIN_6, // D12-PA6
    .SCK_Port = GPIOA,
    .SCK_Pin = GPIO_PIN_7, // D11-PA7

};

#define MAX_SAMPLES 2000 // adjust depending on run length and sample rate

uint32_t time_log[MAX_SAMPLES];
float force_log[MAX_SAMPLES];
uint16_t sample_count = 0;

//  ----------------------------------

// Forward declarations if you keep them elsewhere
void SystemClock_Config(void)
{
    RCC_ClkInitTypeDef RCC_ClkInitStruct;
    RCC_OscInitTypeDef RCC_OscInitStruct;
    HAL_StatusTypeDef ret = HAL_OK;

    /* Enable Power Control clock */
    __HAL_RCC_PWR_CLK_ENABLE();

    /* The voltage scaling allows optimizing the power consumption when the device is
       clocked below the maximum system frequency, to update the voltage scaling value
       regarding system frequency refer to product datasheet.  */
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /* Enable HSI Oscillator and activate PLL with HSI as source */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = 0x10;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM = 16;
    RCC_OscInitStruct.PLL.PLLN = 360;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 7;
    RCC_OscInitStruct.PLL.PLLR = 6;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    /* Activate the OverDrive to reach the 180 MHz Frequency */
    ret = HAL_PWREx_EnableOverDrive();
    if (ret != HAL_OK)
    {
        while (1)
        {
            ;
        }
    }
    /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2
       clocks dividers */
    RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
    {
        Error_Handler();
    }
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
static void MX_USART2_UART_Init(void);
int _write(int file, char *data, int len);

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_USART2_UART_Init(); // 9600 right now
    setbuf(stdout, NULL);  // disable buffering for stdout
    HAL_Delay(1000);       // give the monitor a moment after reset
    printf("\r\nSerial Monitor Awake!\r\n");
    // const char *raw = "RAW\r\n";
    // HAL_UART_Transmit(&huart2, (uint8_t *)raw, 5, 100);

    // Enable GPIO clocks BEFORE calling Motors_Init (Motor_Init uses GPIOB/GPIOA)
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    // Initialize motor HAL and load cell HAL
    Motors_Init();

    // Initialize load cell HAL
    LoadCell_Init(&cell_1, 128);
    // LoadCell_Tare(&cell_1, 16);
    //  Run calibration with known weight
    CalibrateLoadCell(&cell_1, 9.81f); // 1 kg weight = 9.81 N

    // Perform load cell health check
    if (!LoadCell_HealthCheck(&cell_1))
    {
        printf("Health check failed, aborting motor run!\r\n");
        while (1)
        {
            HAL_Delay(500);
        } // halt or blink LED
    }

    // FOR MOTOR-SPACE COMMANDS
    /*
    // Compute signed angle from RPM and time
    // revolutions = rpm * (seconds / 60), angle = 2*pi*revs
    double revs = TARGET_RPM * (RUN_TIME_SEC / 60.0);
    double angle = revs * 2.0 * M_PI;
    if (!CCW_DIRECTION)
        angle = -angle;

    printf("\r\nCMD: rpm=%.1f, t=%.2fs -> revs=%.3f, angle=%.1f deg\r\n",
           (double)TARGET_RPM, (double)RUN_TIME_SEC, revs, (angle * 180.0 / M_PI));
    */

    // FOR OUTPUT-SPACE COMMAND
    printf("\nOutput-Space Commands");
    // printf("\r\nCMD: rpm=%.1f,  angle=%.1f deg\r\n",(double)TARGET_RPM, (double)output_angle);

    uint32_t t0_ms = HAL_GetTick(); // just before starting the move

    // FUNCTION CALLS
    //(void)MoveByAngle(&motor1, angle, TARGET_RPM);
    //(void)MoveByAngleConst(&motor1, angle, TARGET_RPM);
    //(void)MoveByOutputAngle(&motor1, output_angle * (M_PI / 180.0), TARGET_RPM); // func call to desired output angle - output space
    // RepeatabilityTest_OutputCW_FixedRPM(&motor1, cycles, retreatDeg, TARGET_RPM);
    BacklashTest_FixedRPM(&motor1, cycles, retreatDeg, TARGET_RPM);

    // Block until the motion completes (StepMotor() clears isMoving at the end)
    while (motor1.isMoving)
    {
        if (LoadCell_DataReady(&cell_1) && sample_count < MAX_SAMPLES)
        {
            uint32_t t = HAL_GetTick();
            float F = LoadCell_ReadNewton(&cell_1);

            // Store values
            time_log[sample_count] = t;
            force_log[sample_count] = F;
            sample_count++;
        }

        HAL_Delay(1);
    }

    // Optionally stop timers defensively
    StopMotors();

    uint32_t t1_ms = HAL_GetTick(); // right after it finishes
    double elapsed_s = (t1_ms - t0_ms) / 1000.0;

    /*
    printf("DONE: motor stopped. Final revs=%.3f, angle=%.1f deg\r\n",
           (double)revs, (double)(angle * 180.0 / M_PI));
    */
    printf("Elapsed = %.3f s\r\n", elapsed_s);

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
