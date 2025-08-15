#include "peripherals.h"
#include <string.h>

UART_HandleTypeDef huart2;

void SystemClock_Config(void)
{
    // Minimal: leave default HSI clock. Add PLL later if you want.
}

void MX_GPIO_Init(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    GPIO_InitTypeDef g = {0};

    // LED PA5
    g.Pin = LED_PIN;
    g.Mode = GPIO_MODE_OUTPUT_PP;
    g.Pull = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LED_GPIO_PORT, &g);

    // Button PC13 (active low on Nucleo)
    g.Pin = BUTTON_PIN;
    g.Mode = GPIO_MODE_INPUT;
    g.Pull = GPIO_NOPULL; // or GPIO_PULLUP
    HAL_GPIO_Init(BUTTON_GPIO_PORT, &g);
}

void MX_USART2_UART_Init(void)
{
    __HAL_RCC_USART2_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitTypeDef pg = {0};
    // PA2=TX, PA3=RX
    pg.Pin = GPIO_PIN_2 | GPIO_PIN_3;
    pg.Mode = GPIO_MODE_AF_PP;
    pg.Pull = GPIO_PULLUP;
    pg.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    pg.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &pg);

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
        while (1)
        {
        } // simple error trap
    }
}

// Provide the HAL millisecond tick for HAL_Delay()
void SysTick_Handler(void)
{
    HAL_IncTick();
    // HAL_SYSTICK_IRQHandler(); // optional depending on HAL version
}
