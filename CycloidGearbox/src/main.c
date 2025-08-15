#include "peripherals.h"
#include <string.h>

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_USART2_UART_Init();

    const char msg[] = "Button pushed\r\n";

    while (1)
    {
        if (HAL_GPIO_ReadPin(BUTTON_GPIO_PORT, BUTTON_PIN) == GPIO_PIN_RESET)
        {
            HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
            HAL_GPIO_TogglePin(LED_GPIO_PORT, LED_PIN);
            HAL_Delay(300); // debounce
        }
    }
}
