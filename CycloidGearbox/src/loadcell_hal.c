#include "stm32f4xx_hal.h"
#include "loadcell_hal.h"

// Optional tiny delay to stretch SCK high time; tune as needed.
static inline void _short_hold(void)
{
    __NOP();
    __NOP();
    __NOP();
    __NOP();
}

static inline void _pulse(const LoadCell *hx)
{
    HAL_GPIO_WritePin(hx->SCK_Port, hx->SCK_Pin, GPIO_PIN_SET);
    _short_hold();
    HAL_GPIO_WritePin(hx->SCK_Port, hx->SCK_Pin, GPIO_PIN_RESET);
}

void LoadCell_Init(LoadCell *hx, uint8_t gain)
{
    GPIO_InitTypeDef gi = {0};

    // Configure DOUT as input
    gi.Pin = hx->D_Pin;
    gi.Mode = GPIO_MODE_INPUT;
    gi.Pull = GPIO_NOPULL; // or GPIO_PULLUP if noisy
    HAL_GPIO_Init(hx->D_Port, &gi);

    // Configure SCK as output, idle low
    gi.Pin = hx->SCK_Pin;
    gi.Mode = GPIO_MODE_OUTPUT_PP;
    gi.Pull = GPIO_NOPULL;
    gi.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(hx->SCK_Port, &gi);
    HAL_GPIO_WritePin(hx->SCK_Port, hx->SCK_Pin, GPIO_PIN_RESET);

    // Map gain → extra pulses
    switch (gain)
    {
    case 128:
        hx->gain_pulses = 1;
        break; // A,128
    case 64:
        hx->gain_pulses = 3;
        break; // A,64
    case 32:
        hx->gain_pulses = 2;
        break; // B,32
    default:
        hx->gain_pulses = 1;
        break; // Default to A,128
    }

    // Reset calibration state
    hx->offset = 0;
    hx->scale = 1.0f;
    hx->ready = 0;
    hx->last_raw = 0;
}

int LoadCell_DataReady(LoadCell *hx)
{
    // DOUT goes LOW when a new conversion is ready
    return (HAL_GPIO_ReadPin(hx->D_Port, hx->D_Pin) == GPIO_PIN_RESET);
}

int32_t LoadCell_ReadRaw(LoadCell *hx)
{
    // Wait for DRDY (DOUT low)
    while (!LoadCell_DataReady(hx))
    { /* HAL_Delay(1); */
    }

    uint32_t v = 0;
    for (int i = 0; i < 24; i++) // 24 SCK pulses
    {
        _pulse(hx);
        v = (v << 1) | (HAL_GPIO_ReadPin(hx->D_Port, hx->D_Pin) == GPIO_PIN_SET);
    }

    // Extra pulses to select next conversion's channel/gain
    for (uint8_t i = 0; i < hx->gain_pulses; i++)
    {
        _pulse(hx);
    }

    // Sign-extend 24-bit two's complement to 32-bit
    if (v & 0x800000U)
    {
        v |= 0xFF000000U;
    }
    hx->last_raw = (int32_t)v;
    return hx->last_raw;
}

int32_t LoadCell_ReadAverage(LoadCell *hx, uint8_t n)
{
    int64_t sum = 0;
    if (n == 0)
        n = 1;
    for (uint8_t i = 0; i < n; i++)
    {
        sum += LoadCell_ReadRaw(hx);
    }
    return (int32_t)(sum / (int32_t)n);
}

void LoadCell_Tare(LoadCell *hx, uint8_t n)
{
    hx->offset = LoadCell_ReadAverage(hx, n);
}

void LoadCell_SetScale(LoadCell *hx, float counts_per_N)
{
    if (counts_per_N == 0.0f)
        counts_per_N = 1.0f;
    hx->scale = counts_per_N;
}

float LoadCell_ReadNewton(LoadCell *hx)
{
    // Average a few samples to reduce noise
    int32_t avg = LoadCell_ReadAverage(hx, 8);
    return (avg - hx->offset) / hx->scale;
}

int LoadCell_HealthCheck(LoadCell *hx)
{
    // 1. Check if DOUT ever goes low (data ready)
    uint32_t start = HAL_GetTick();
    while (HAL_GetTick() - start < 200)
    { // wait up to 200 ms
        if (LoadCell_DataReady(hx))
        {
            break;
        }
    }
    if (!LoadCell_DataReady(hx))
    {
        printf("LoadCell ERROR: No data ready\r\n");
        return 0;
    }

    // 2. Take a few samples
    int32_t s1 = LoadCell_ReadRaw(hx);
    HAL_Delay(50);
    int32_t s2 = LoadCell_ReadRaw(hx);

    // 3. Sanity check values
    if (s1 == 0x7FFFFF || s1 == -8388608)
    { // extremes = open/shorted
        printf("LoadCell ERROR: Out of range\r\n");
        return 0;
    }
    if (s1 == s2)
    {
        printf("LoadCell WARNING: No variation detected\r\n");
    }

    printf("LoadCell OK: raw=%ld..%ld\r\n", (long)s1, (long)s2);
    return 1;
}

void CalibrateLoadCell(LoadCell *hx, float known_force_N)
{
    printf("Taring... remove all load.\r\n");
    LoadCell_Tare(hx, 16);
    HAL_Delay(500);

    printf("Place %.2f N load and press reset or rerun.\r\n", known_force_N);

    // Wait a bit for user to place weight
    HAL_Delay(5000);

    int32_t raw = LoadCell_ReadAverage(hx, 16);
    float scale = (raw - hx->offset) / known_force_N;
    LoadCell_SetScale(hx, scale);

    printf("Calibration done.\r\n");
    printf("Raw with load = %ld\r\n", (long)raw);
    printf("Scale factor (counts per N) = %.3f\r\n", scale);
}
