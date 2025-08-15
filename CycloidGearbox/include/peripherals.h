#ifndef PERIPHERALS_H
#define PERIPHERALS_H

#include "stm32f4xx_hal.h"

// Pin names
#define LED_GPIO_PORT GPIOA
#define LED_PIN GPIO_PIN_5
#define BUTTON_GPIO_PORT GPIOC
#define BUTTON_PIN GPIO_PIN_13

extern UART_HandleTypeDef huart2;

void SystemClock_Config(void);
void MX_GPIO_Init(void);
void MX_USART2_UART_Init(void);

// SysTick handler declaration (definition in peripherals.c)
void SysTick_Handler(void);

#endif
