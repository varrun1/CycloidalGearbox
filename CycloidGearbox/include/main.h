#ifndef __MAIN_H
#define __MAIN_H

#include "stm32f4xx_hal.h"
#include "motor_hal.h" // exposes Motor, Motors_Init, MoveByAngle, etc.

#ifdef __cplusplus
extern "C"
{
#endif

    void SystemClock_Config(void);
    void Error_Handler(void);

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
