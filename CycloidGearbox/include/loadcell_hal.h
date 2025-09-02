#pragma once
#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdio.h>

typedef struct
{
    // const char *name;
    GPIO_TypeDef *D_Port;   // Port of the motor
    uint16_t D_Pin;         // Pin for stepping
    GPIO_TypeDef *SCK_Port; // Port for the direction
    uint16_t SCK_Pin;       // Pin to set direction

    uint8_t gain_pulses;    // 1=128(A), 2=64(A), 3=32(B)
    int32_t offset;         // from tare
    float scale;            // counts per N (or per kg)
    volatile uint8_t ready; // set by EXTI or polling

    int32_t last_raw; // optional cache

} LoadCell;

extern LoadCell cell_1;

void LoadCell_Init(LoadCell *hx, uint8_t gain);

int LoadCell_DataReady(LoadCell *hx);                     // 1 when DOUT goes low
int32_t LoadCell_ReadRaw(LoadCell *hx);                   // blocking 24-bit read
int32_t LoadCell_ReadAverage(LoadCell *hx, uint8_t n);    // simple noise reduction
void LoadCell_Tare(LoadCell *hx, uint8_t n);              // store zero offset
void LoadCell_SetScale(LoadCell *hx, float counts_per_N); // or counts_per_kg
float LoadCell_ReadNewton(LoadCell *hx);                  // scaled, offset-corrected
int LoadCell_HealthCheck(LoadCell *hx);
void CalibrateLoadCell(LoadCell *hx, float known_force_N);