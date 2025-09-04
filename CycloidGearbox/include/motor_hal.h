#ifndef __MOTOR_HAL_H
#define __MOTOR_HAL_H

#include "stdio.h"
#include <math.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include "stm32f4xx_hal_tim.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define STEPS_PER_REV 3200 // 1/16th stepping
#define MIN_RPM 50
#define MAX_RPM 800
#define Output_Reduction 29

#define CW 0
#define CCW 1

typedef struct
{
    const char *name;
    GPIO_TypeDef *stepPort;   // Port of the motor
    uint16_t stepPin;         // Pin for stepping
    GPIO_TypeDef *dirPort;    // Port for the direction
    uint16_t dirPin;          // Pin to set direction
    bool dir;                 // Motor direction
    double reduction;         // Gear reduction of the motor
    uint32_t stepsToComplete; // Number of steps the motor has left to complete
    uint32_t stepsToSpeedUp;  // How many steps the motor has to ramp up speed
    uint32_t stepsToSlowDown; // How many steps the motor has to ramp down speed
    double slope;             // The slope betweent the min and target speed
    double currentRPM;        // The motors current rpm
    bool isMoving;            // Is the motor moving?

} Motor;

extern Motor motor1;

void Motors_Init(void);
double MoveByAngle(Motor *motor, double angle, double speedRPM);
double MoveByAngleConst(Motor *motor, double angle, double speedRPM);
double MoveByOutputAngle(Motor *motor, double angle, double speedRPM);
double MoveOutputByDeg_FixedMotorRPM(Motor *m, double out_deg, double motor_rpm);
void Landing_FromCW(Motor *m, double retreat_deg, double motor_rpm);
void Landing_FromCCW(Motor *m, double retreat_deg, double motor_rpm);

void RepeatabilityTest_OutputCW_FixedRPM(Motor *m, int cycles, double retreat_deg, double motor_rpm);
void BacklashTest_FixedRPM(Motor *m, int cycles, double retreat_deg, double motor_rpm);

void HomeMotors(void);
void StopMotors(void);
void Error_Handler(void);

#endif