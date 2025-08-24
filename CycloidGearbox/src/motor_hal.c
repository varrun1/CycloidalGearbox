#include "stm32f4xx_hal.h"
#include "motor_hal.h"
// #include "controls.h"

ADC_HandleTypeDef hadc1;
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim7;

void Motor_Init(Motor motor);
void StepMotor(Motor *motor);
static void TIM3_Init(void);
// static void TIM4_Init(void);
// static void TIM7_Init(void);
void TIM3_IRQHandler(void);
// void TIM4_IRQHandler(void);
// void TIM7_IRQHandler(void);
//  static void MX_ADC1_Init(void); // Not currently used, can delete later
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
    // TIM4_Init();
    // TIM7_Init();

    MX_TIM2_Init();
    // MX_ADC1_Init();

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

    /*
    // If we are in manual, set speed to desired speed right away
    if (state.manual)
    {
        motor->currentRPM = speedRPM;
    }
    else
    {
        motor->currentRPM = MIN_RPM;
    }
    */

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
    // If your TIM3 tick = 1 µs, ARR is in microseconds.
    // One full STEP = high + low => two toggles per step:
    float timePerStep_s = 60.0f / (motor->currentRPM * STEPS_PER_REV * motor->reduction);
    uint32_t timerPeriod = (uint32_t)((timePerStep_s * 1e6f) / 2.0f) - 1u;

    // (Optional) clamp to keep a minimum pulse width
    if (timerPeriod < 50u)
        timerPeriod = 50u;

    // Signed angle actually commanded (for your return)
    double angleToComplete = (motor->stepsToComplete / (double)STEPS_PER_REV) / motor->reduction * 2.0 * M_PI;
    if (motor->dir == CW)
        angleToComplete = -angleToComplete;

    // Program timer and start
    // (Consider resetting the counter before start to get a clean first pulse)
    if (motor->name == motor1.name)
    { // (see note below)
        __HAL_TIM_SET_AUTORELOAD(&htim3, timerPeriod);
        __HAL_TIM_SET_COUNTER(&htim3, 0);
        HAL_TIM_Base_Start_IT(&htim3);
    }

    return angleToComplete;
}

/*
double MoveByDist(Motor *motor, double dist, double speedRPM)
{
    if (dist > 0)
    {
        HAL_GPIO_WritePin(motor->dirPort, motor->dirPin, CCW);
        motor->dir = CCW;
    }
    else
    {
        HAL_GPIO_WritePin(motor->dirPort, motor->dirPin, CW);
        motor->dir = CW;
        dist = dist * -1;
    }
    double revs = dist / Z_MM_PER_REV;
    motor->stepsToComplete = (uint32_t)(revs * Z_STEPS_PER_REV);

    // Gain scheduling setup
    // Speed up for first 1/4 steps
    motor->stepsToSpeedUp = 3.0 / 4.0 * motor->stepsToComplete;
    // Slow down for last 1/4 steps
    motor->stepsToSlowDown = 1.0 / 4.0 * motor->stepsToComplete;
    // RPM delta per step
    motor->slope = (speedRPM - MIN_RPM) / (motor->stepsToSlowDown);
    // Start at the min rpm
    motor->currentRPM = MIN_RPM;

    // If we are in manual, set speed to desired speed right away
    if (state.manual)
    {
        motor->currentRPM = speedRPM;
    }
    else
    {
        motor->currentRPM = MIN_RPM;
    }

    float timePerStep = 60.0 / (motor->currentRPM * Z_STEPS_PER_REV);   // Time per step in seconds
    uint32_t timerPeriod = (uint32_t)((timePerStep * 1000000) / 2) - 1; // Time per toggle, in microseconds
    motor->isMoving = 1;

    double distToComplete = motor->stepsToComplete / Z_STEPS_PER_REV * Z_MM_PER_REV;
    if (motor->dir == CW)
    {
        distToComplete = distToComplete * -1;
    }

    if (motor->name == motorz.name)
    {
        __HAL_TIM_SET_AUTORELOAD(&htim7, timerPeriod);
        HAL_TIM_Base_Start_IT(&htim7);
    }

    return distToComplete;
}
*/

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

    // Gain scheduling (unchanged)
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

    if (motor->name == motor1.name)
    {
        __HAL_TIM_SET_AUTORELOAD(&htim3, (uint32_t)timerPeriod);
        __HAL_TIM_SET_COUNTER(&htim3, 0); // helps apply new ARR cleanly
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
    HAL_TIM_Base_Init(&htim3);

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

/*
static void TIM4_Init(void)
{
    __HAL_RCC_TIM4_CLK_ENABLE();

    htim4.Instance = TIM4;
    htim4.Init.Prescaler = (uint32_t)((SystemCoreClock / 1000000) - 1); // 1 MHz clock
    htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim4.Init.Period = 0xFFFF; // Max value, update frequency will be set in stepMotor()
    htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_Base_Init(&htim4);

    HAL_NVIC_SetPriority(TIM4_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM4_IRQn);
}

void TIM4_IRQHandler(void)
{
    if (__HAL_TIM_GET_FLAG(&htim4, TIM_FLAG_UPDATE) != RESET)
    {
        if (__HAL_TIM_GET_IT_SOURCE(&htim4, TIM_IT_UPDATE) != RESET)
        {
            __HAL_TIM_CLEAR_IT(&htim4, TIM_IT_UPDATE);
            StepMotor(&motor2);
        }
    }
}

static void TIM7_Init(void)
{
    __HAL_RCC_TIM7_CLK_ENABLE();

    htim7.Instance = TIM7;
    htim7.Init.Prescaler = (uint32_t)((SystemCoreClock / 1000000) - 1); // 1 MHz clock
    htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim7.Init.Period = 0xFFFF; // Max value, update frequency will be set in stepMotor()
    htim7.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_Base_Init(&htim7);

    HAL_NVIC_SetPriority(TIM7_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM7_IRQn);
}

void TIM7_IRQHandler(void)
{
    if (__HAL_TIM_GET_FLAG(&htim7, TIM_FLAG_UPDATE) != RESET)
    {
        if (__HAL_TIM_GET_IT_SOURCE(&htim7, TIM_IT_UPDATE) != RESET)
        {
            __HAL_TIM_CLEAR_IT(&htim7, TIM_IT_UPDATE);
            StepMotor(&motorz);
        }
    }
}
*/

/**
 * @brief Will stop all motors immediately.
 *
 */
void StopMotors(void)
{
    HAL_TIM_Base_Stop_IT(&htim3);
    // HAL_TIM_Base_Stop_IT(&htim4);
    // HAL_TIM_Base_Stop_IT(&htim7);
}

/**
 * @brief Homes the motors.
 *
 */
/*
void HomeMotors(void)
{
    printf("Homing...\n\r");
    updateStateMachine("Homing");

    gripperClose(&gripper);
    MoveByDist(&motorz, -125.0, 20.0);
    MoveByAngle(&motor1, 8.0, 5.0);
    MoveByAngle(&motor2, 8.0, 5.0);

    while (motor1.isMoving || motor2.isMoving || motorz.isMoving)
    {
        HAL_Delay(1);
    }
    HAL_Delay(1000);

    // Move back 6 degrees
    double distZ = MoveByDist(&motorz, 2.0, 5.0);
    double theta1 = MoveByAngle(&motor1, -6.0 / 180.0 * M_PI, 1.0);
    double theta2 = MoveByAngle(&motor2, -6.0 / 180.0 * M_PI, 1.0);

    while (motor1.isMoving || motor2.isMoving || motorz.isMoving)
    {
        HAL_Delay(1);
    }

    // Update the state machine
    updateStateMachine("Auto Wait");
    state.theta1 = motor1.thetaMax + theta1;
    state.theta2 = motor2.thetaMax + theta2;
    state.currentZ = motorz.thetaMin + distZ;
    CalculateCartesianCoords(state.theta1, state.theta2, &state.x, &state.y);
    printf("Current Coords in x-y:");
    PrintCaresianCoords(state.x, state.y);
}
*/

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