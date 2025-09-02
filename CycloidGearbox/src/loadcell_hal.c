#include "stm32f4xx_hal.h"
#include "loadcell_hal.h"
// #include "controls.h"

void LoadCell_Init(void);

LoadCell cell_1 = {
    .D_Port = GPIOA,
    .D_Pin = GPIO_PIN_6, // D12-PA6
    .SCK_Port = GPIOA,
    .SCK_Pin = GPIO_PIN_7, // D11-PA7

};