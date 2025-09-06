#include "stm32f4xx_hal.h"
#include "motor_hal.h"

ADC_HandleTypeDef hadc1;
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim7;

void Motor_Init(Motor motor);
void StepMotor(Motor *motor);
static void TIM3_Init(void);
void TIM3_IRQHandler(void);
static void MX_TIM2_Init(void);

// Motor Objects
Motor motor1 = {
    // X
    .name = "motor1",
    .stepPort = GPIOB,     // D5-PB4
    .stepPin = GPIO_PIN_4, // D5-PB4
    .dirPort = GPIOB,      // D4-PB5
    .dirPin = GPIO_PIN_5,  // D4-PB5
    .dir = CCW,
    .reduction = 1,
    .isMoving = 0,
};

/**
 * @brief Takes in a motor struct and initialized the associated pins.
 *
 * @param motor Motor struct
 */
void Motor_Init(Motor motor)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // Initialize Step Pin
    GPIO_InitStruct.Pin = motor.stepPin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(motor.stepPort, &GPIO_InitStruct);

    // Initialize Direction Pin
    GPIO_InitStruct.Pin = motor.dirPin;
    HAL_GPIO_Init(motor.dirPort, &GPIO_InitStruct);
}

/**
 * @brief Initializes the motor and start the timer.
 *
 */
void Motors_Init(void)
{
    Motor_Init(motor1);
    TIM3_Init();

    // --- Force TIM3 to 1 MHz tick ---
    uint32_t timclk = HAL_RCC_GetPCLK1Freq();
    RCC_ClkInitTypeDef clk;
    uint32_t tmp;
    HAL_RCC_GetClockConfig(&clk, &tmp);
    if (clk.APB1CLKDivider != RCC_HCLK_DIV1)
        timclk *= 2U; // APB1 timers double

    uint32_t psc_for_10MHz = (uint32_t)((timclk + 500000U) / 1000000U) - 1U; // round to nearest
    __HAL_TIM_SET_PRESCALER(&htim3, psc_for_10MHz);

    // Ensure PSC/ARR load immediately (ARR can be any placeholder here)
    __HAL_TIM_SET_AUTORELOAD(&htim3, 1000 - 1); // dummy ARR
    __HAL_TIM_SET_COUNTER(&htim3, 0);
    __HAL_TIM_CLEAR_FLAG(&htim3, TIM_FLAG_UPDATE);
    htim3.Instance->EGR = TIM_EGR_UG; // latch PSC/ARR now
    // ---------------------------------------------

    uint32_t psc = htim3.Instance->PSC;
    double tick_us = 1e6 * (double)(psc + 1) / (double)timclk;
    printf("TIM3 tick = %.3f us (PSC=%lu)\r\n", tick_us, (unsigned long)psc);

    MX_TIM2_Init();

    if (HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
 * @brief Move the specified motor.
 *
 * @param motor Motor to move
 * @param angle Angle in radians
 * @param speedRPM Speed in RPM
 * @return double
 */
double MoveByAngle(Motor *motor, double angle, double speedRPM)
{
    motor->isMoving = 1;
    if (angle > 0)
    {
        HAL_GPIO_WritePin(motor->dirPort, motor->dirPin, CCW);
        motor->dir = CCW;
    }
    else
    {
        HAL_GPIO_WritePin(motor->dirPort, motor->dirPin, CW);
        motor->dir = CW;
        angle = angle * -1;
    }

    // Gain scheduling setup
    motor->stepsToComplete = (uint32_t)((angle / (2 * M_PI)) * STEPS_PER_REV * motor->reduction);
    // Speed up for first 1/4 steps
    motor->stepsToSpeedUp = 3.0 / 4.0 * motor->stepsToComplete;
    // Slow down for last 1/4 steps
    motor->stepsToSlowDown = 1.0 / 4.0 * motor->stepsToComplete;
    // RPM delta per step
    motor->slope = (speedRPM - MIN_RPM) / (motor->stepsToSlowDown);
    // Start at the min rpm
    motor->currentRPM = MIN_RPM;

    float timePerStep = 60.0 / (motor->currentRPM * STEPS_PER_REV * motor->reduction); // Time per step in seconds
    uint32_t timerPeriod = (uint32_t)((timePerStep * 1000000) / 2) - 1;                // Time per toggle, in microseconds

    double angleToComplete = motor->stepsToComplete / STEPS_PER_REV / motor->reduction * 2 * M_PI;
    if (motor->dir == CW)
    {
        angleToComplete = angleToComplete * -1;
    }

    if (motor->name == motor1.name)
    {
        __HAL_TIM_SET_AUTORELOAD(&htim3, timerPeriod);
        HAL_TIM_Base_Start_IT(&htim3);
    }

    return angleToComplete;
}

double MoveByAngleConst(Motor *motor, double angle, double speedRPM)
{
    motor->isMoving = 1;

    // Direction (your convention: +angle => CCW)
    if (angle > 0)
    {
        HAL_GPIO_WritePin(motor->dirPort, motor->dirPin, CCW);
        motor->dir = CCW;
    }
    else
    {
        HAL_GPIO_WritePin(motor->dirPort, motor->dirPin, CW);
        motor->dir = CW;
        angle = -angle; // use magnitude below
    }

    // Steps to move (same as before)
    motor->stepsToComplete = (uint32_t)((angle / (2.0 * M_PI)) * STEPS_PER_REV * motor->reduction);

    // ---- Disable gain scheduling / ramp completely ----
    if (speedRPM < MIN_RPM)
        speedRPM = MIN_RPM;
    motor->currentRPM = speedRPM;       // fixed speed
    motor->stepsToSpeedUp = UINT32_MAX; // accel branch can never trigger
    motor->stepsToSlowDown = 0;         // decel branch can never trigger
    motor->slope = 0.0;                 // no change per step
    // ---------------------------------------------------

    // Timer period for constant step rate.
    // One full STEP = high + low => two toggles per step:
    float timePerStep_s = 60.0f / (motor->currentRPM * STEPS_PER_REV * motor->reduction);
    uint32_t timerPeriod = (uint32_t)((timePerStep_s * 1e6f) / 2.0f) - 1u;

    // clamp to keep a minimum pulse width
    if (timerPeriod < 50u)
        timerPeriod = 50u;

    // Signed angle actually commanded
    double angleToComplete = (motor->stepsToComplete / (double)STEPS_PER_REV) / motor->reduction * 2.0 * M_PI;
    if (motor->dir == CW)
        angleToComplete = -angleToComplete;

    // Program timer and start
    if (motor->name == motor1.name)
    {
        __HAL_TIM_SET_AUTORELOAD(&htim3, timerPeriod);
        __HAL_TIM_SET_COUNTER(&htim3, 0);
        HAL_TIM_Base_Start_IT(&htim3);
    }

    return angleToComplete;
}

/// @brief Move output disc by desired angle
/// @param motor Motor to move
/// @param angle angle in radians
/// @param speedRPM desired motor speed in RPM
/// @return double
double MoveByOutputAngle(Motor *motor, double angle, double speedRPM)
{
    motor->isMoving = 1;

    // Direction (your convention: +angle => CCW)
    if (angle > 0)
    {
        HAL_GPIO_WritePin(motor->dirPort, motor->dirPin, CCW);
        motor->dir = CCW;
    }
    else
    {
        HAL_GPIO_WritePin(motor->dirPort, motor->dirPin, CW);
        motor->dir = CW;
        angle = -angle; // use magnitude below
    }

    // Steps to move (same as before)
    double steps_f = (angle / (2.0 * M_PI)) * (STEPS_PER_REV * Output_Reduction);
    if (steps_f < 0.5)
        steps_f = 0.0;
    motor->stepsToComplete = (uint32_t)(steps_f + 0.5);

    // ---- Disable gain scheduling / ramp completely ----
    if (speedRPM < MIN_RPM)
        speedRPM = MIN_RPM;
    if (speedRPM > MAX_RPM)
        speedRPM = MAX_RPM;
    motor->currentRPM = speedRPM;       // fixed speed
    motor->stepsToSpeedUp = UINT32_MAX; // accel branch can never trigger
    motor->stepsToSlowDown = 0;         // decel branch can never trigger
    motor->slope = 0.0;                 // no change per step
    // ---------------------------------------------------

    // Timer period for constant step rate.
    float timePerStep_s = 60.0f / (motor->currentRPM * STEPS_PER_REV);
    uint32_t timerPeriod = (uint32_t)((timePerStep_s * 1e6f) / 2.0f) - 1u;

    // clamp to keep a minimum pulse width
    if (timerPeriod < 50u)
        timerPeriod = 50u;

    // Signed angle actually commanded
    double angleToComplete = ((double)motor->stepsToComplete / (STEPS_PER_REV * Output_Reduction)) * 2.0 * M_PI;
    if (motor->dir == CW)
        angleToComplete = -angleToComplete;

    // Program timer and start
    if (motor->name == motor1.name)
    {
        __HAL_TIM_SET_AUTORELOAD(&htim3, timerPeriod);
        __HAL_TIM_SET_COUNTER(&htim3, 0);
        HAL_TIM_Base_Start_IT(&htim3);
    }

    return angleToComplete;
}

// ----- Repeatability (constant motor RPM, output-space commands) -----
static void wait_until_done(Motor *m)
{
    while (m->isMoving)
        HAL_Delay(1);
}

double MoveOutputByDeg_FixedMotorRPM(Motor *m, double out_deg, double motor_rpm)
{
    // Convert OUTPUT angle (deg) to MOTOR angle (rad)
    double motor_angle_rad = (out_deg * (M_PI / 180.0)) * Output_Reduction;

    // Clamp RPM
    double rpm = motor_rpm;
    if (rpm < MIN_RPM)
        rpm = MIN_RPM;
    if (rpm > MAX_RPM)
        rpm = MAX_RPM;

    // Execute constant-speed move in motor-space
    return MoveByAngleConst(m, motor_angle_rad, rpm);
}

// One landing from CW approach.
// PRECONDITION: already touching the probe (target), indicator zeroed.
void Landing_FromCW(Motor *m, double retreat_deg, double motor_rpm)
{
    // 1) Back off CCW (+retreat)
    printf("   Back off CCW +%.3f deg\r\n", fabs(retreat_deg));
    (void)MoveOutputByDeg_FixedMotorRPM(m, +fabs(retreat_deg), motor_rpm);
    wait_until_done(m);
    HAL_Delay(300); // settle

    // 2) Re-approach CW (−retreat) to land again from CW
    printf("   Re-approach CW -%.3f deg (land)\r\n", fabs(retreat_deg));
    (void)MoveOutputByDeg_FixedMotorRPM(m, -fabs(retreat_deg), motor_rpm);
    wait_until_done(m);
    HAL_Delay(1000); // settle, then read the dial
}

void Landing_FromCCW(Motor *m, double retreat_deg, double motor_rpm)
{
    // Back off CW (-retreat)
    printf("   Back off CW  -%.3f deg\n", fabs(retreat_deg));
    (void)MoveOutputByDeg_FixedMotorRPM(m, -fabs(retreat_deg), motor_rpm);
    wait_until_done(m);
    HAL_Delay(300);

    // Re-approach CCW (+retreat) to land from CCW
    printf("   Approach CCW +%.3f deg  [LAND]\n", fabs(retreat_deg));
    (void)MoveOutputByDeg_FixedMotorRPM(m, +fabs(retreat_deg), motor_rpm);
    wait_until_done(m);
    HAL_Delay(1000); // read dial now
}

// Multiple landings for repeatability stats.
// Start ON the probe (zeroed). Each cycle gives one CW landing.
void RepeatabilityTest_OutputCW_FixedRPM(Motor *m, int cycles, double retreat_deg, double motor_rpm)
{
    printf("\r\n=== Repeatability (CW approach, fixed motor RPM) ===\r\n");
    printf("Start: ON probe, zeroed. cycles=%d, retreat=%.3f deg, motorRPM=%.1f, ratio=%.3f\r\n",
           cycles, retreat_deg, motor_rpm, (double)Output_Reduction);

    for (int i = 1; i <= cycles; ++i)
    {
        printf("\r\n[CYCLE %d]\r\n", i);
        Landing_FromCW(m, retreat_deg, motor_rpm);
        // Read indicator here
    }
    printf("\r\n=== Test complete ===\r\n");
}

void BacklashTest_FixedRPM(Motor *m, int cycles, double retreat_deg, double motor_rpm)
{
    // const double mm_to_arcmin = 10800.0 / (M_PI * radius_mm);

    printf("\n=== Backlash Test (fixed motor RPM) ===\n");
    printf("Start ON probe, zeroed. cycles=%d, retreat=%.3f deg, motorRPM=%.1f\n", cycles, retreat_deg, motor_rpm);
    printf("Sequence per cycle: CW-landing (read A), CCW-landing (read B) → backlash = |A-B|.\n");

    for (int i = 1; i <= cycles; ++i)
    {
        printf("\n[CYCLE %d]\n", i);

        printf(" CW-landing:\n");
        Landing_FromCW(m, retreat_deg, motor_rpm);
        // >>> Read dial here → record A_i (mm)
        printf("Read dial here → record A_i (mm)\n");

        printf(" CCW-landing:\n");
        Landing_FromCCW(m, retreat_deg, motor_rpm);
        // >>> Read dial here → record B_i (mm)
        printf("Read dial here → record B_i (mm)\n");
    }

    printf("\n=== Backlash Test complete ===\n");
}

void StepMotor(Motor *motor)
{
    // Stop condition — exit immediately to avoid underflow & extra toggles
    if (!motor->stepsToComplete || !motor->isMoving)
    {
        if (motor->name == motor1.name) // (see note D below)
        {
            HAL_TIM_Base_Stop_IT(&htim3);
        }
        motor->isMoving = 0;
        return; // <-- critical
    }

    // Gain scheduling
    if (motor->stepsToComplete > motor->stepsToSpeedUp)
        motor->currentRPM += motor->slope;
    else if (motor->stepsToComplete < motor->stepsToSlowDown)
        motor->currentRPM -= motor->slope;

    // Recompute period
    float timePerStep = 60.0f / (motor->currentRPM * STEPS_PER_REV * motor->reduction);
    int32_t timerPeriod = (int32_t)((timePerStep * 1000000.0f) / 2.0f) - 1;
    if (timerPeriod < 1)
        timerPeriod = 1;
    if (timerPeriod > 0xFFFF)
        timerPeriod = 0xFFFF;

    if (motor == &motor1)
    {
        uint32_t newARR = (uint32_t)timerPeriod;
        if (htim3.Instance->ARR != newARR)
        {
            __HAL_TIM_SET_AUTORELOAD(&htim3, newARR);
        }
    }

    // Generate the step and count ONLY on rising edges
    HAL_GPIO_TogglePin(motor->stepPort, motor->stepPin);
    if (HAL_GPIO_ReadPin(motor->stepPort, motor->stepPin) == GPIO_PIN_SET)
    {
        motor->stepsToComplete--;
    }
}

static void TIM3_Init(void)
{
    __HAL_RCC_TIM3_CLK_ENABLE();
    htim3.Instance = TIM3;
    htim3.Init.Prescaler = (uint32_t)((SystemCoreClock / 1000000) - 1);
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim3.Init.Period = 0xFFFF;
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    HAL_TIM_Base_Init(&htim3);

    __HAL_TIM_SET_COUNTER(&htim3, 0);
    __HAL_TIM_CLEAR_FLAG(&htim3, TIM_FLAG_UPDATE);
    htim3.Instance->EGR = TIM_EGR_UG;

    HAL_NVIC_SetPriority(TIM3_IRQn, 5, 0); // was (0,0)
    HAL_NVIC_EnableIRQ(TIM3_IRQn);
}

void TIM3_IRQHandler(void)
{
    if (__HAL_TIM_GET_FLAG(&htim3, TIM_FLAG_UPDATE) != RESET)
    {
        if (__HAL_TIM_GET_IT_SOURCE(&htim3, TIM_IT_UPDATE) != RESET)
        {
            __HAL_TIM_CLEAR_IT(&htim3, TIM_IT_UPDATE);
            StepMotor(&motor1);
        }
    }
}

/**
 * @brief Will stop all motors immediately.
 *
 */
void StopMotors(void)
{
    HAL_TIM_Base_Stop_IT(&htim3);
}

static void MX_TIM2_Init(void)
{
    TIM_OC_InitTypeDef sConfigOC = {0};

    // Enable clock for TIM2
    __HAL_RCC_TIM2_CLK_ENABLE();

    // Enable clock for GPIOA (if not already enabled)
    __HAL_RCC_GPIOA_CLK_ENABLE();

    htim2.Instance = TIM2;
    // Adjust Prescaler to get a 1 MHz timer clock frequency
    htim2.Init.Prescaler = (uint32_t)((SystemCoreClock / 1000000) - 1); // Now directly using SystemCoreClock
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 19999; // Sets the PWM frequency to 50 Hz (20 ms period)
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_PWM_Init(&htim2); // Initialize TIM2 in PWM mode

    // Configure the PWM channel
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 1500; // Initial pulse width of 1.5 ms, adjust as needed for servo positioning
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1);

    // Configure GPIO Pin PA5 for Alternate Function (TIM2_CH1)
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP; // Push-Pull Alternate Function
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM2; // Ensure this is the correct alternate function for your MCU series
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // Start PWM on TIM2 Channel 1
    // HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
}