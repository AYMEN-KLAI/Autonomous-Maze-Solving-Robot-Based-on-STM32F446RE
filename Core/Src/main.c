/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Maze Solver Robot — STM32F446RE
  *                   Motors : BTS7960 x2 via TIM1
  *                   Left   : FWD=TIM1_CH1(PA8)  BWD=TIM1_CH2(PA9)
  *                   Right  : FWD=TIM1_CH3(PA10) BWD=TIM1_CH4(PA11)
  *                   Enc L  : TIM2 (PA0,PA1) - 32-bit
  *                   Enc R  : TIM3 (PC6,PC7) - 16-bit
  *                   TIM5   : microsecond timer for ultrasonic (32-bit)
  *                   Maze   : 7×7 grid, 40×40 cm cells
  *                   Algo   : Flood-fill explore → BFS sprint
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"
#include "MPU6050.h"
#include "maze.h"
#include <math.h>
#include <string.h>
#include <stdio.h>
#include "stm32f4xx_hal_i2c.h"

/* ============================================================
   PERIPHERAL HANDLES
   ============================================================ */
I2C_HandleTypeDef  hi2c1;
TIM_HandleTypeDef  htim1;   /* PWM — motors               */
TIM_HandleTypeDef  htim2;   /* Encoder — left  (32-bit)   */
TIM_HandleTypeDef  htim3;   /* Encoder — right (16-bit)   */
TIM_HandleTypeDef  htim5;   /* µs timer — ultrasonic      */
UART_HandleTypeDef huart2;

/* ============================================================
   MOTOR CHANNEL MAPPING
   BTS7960: RPWM = forward, LPWM = reverse
   TIM1 CH1/CH2 → Left motor    (PA8 / PA9)
   TIM1 CH3/CH4 → Right motor   (PA10/ PA11)
   ============================================================ */
#define LEFT_RPWM_CH    TIM_CHANNEL_1
#define LEFT_LPWM_CH    TIM_CHANNEL_2
#define RIGHT_RPWM_CH   TIM_CHANNEL_3
#define RIGHT_LPWM_CH   TIM_CHANNEL_4

/* ============================================================
   ROBOT PARAMETERS
   ============================================================ */
#define PWM_MAX         9999
#define PWM_MIN         4000

#define RIGHT_RADIUS_MM         65.0f
#define LEFT_RADIUS_MM          65.0f
#define WHEEL_BASE_MM           280.0f

#define RIGHT_RESOLUTION        3920.0f  /* encoder ticks / rev */
#define LEFT_RESOLUTION         3942.0f

#define T_MS    5            /* control loop period [ms] */
#define T_S     0.005f       /* control loop period [s]  */

static uint32_t imu_last_tick  = 0;
static uint8_t  imu_initialized = 0;

#define KP_DEFAULT               0.08f
#define KI_DEFAULT               0.00f
#define KD_DEFAULT               0.00f
#define RIGHT_WHEEL_GAIN_FORWARD  0.9f
#define RIGHT_WHEEL_GAIN_BACKWARD 1.0f

/* ============================================================
   PID STRUCTURE
   ============================================================ */
typedef struct {
    float kp, ki, kd;
    float integral;
    float prev_error;
} PID_t;

/* ============================================================
   GLOBALS
   ============================================================ */
static const float PI_F = 3.141592653589793f;

PID_t pid_L       = { KP_DEFAULT, KI_DEFAULT, KD_DEFAULT, 0.0f, 0.0f };
PID_t pid_R       = { KP_DEFAULT, KI_DEFAULT, KD_DEFAULT, 0.0f, 0.0f };
PID_t pid_heading = { 2.0f,       0.0f,       0.0f,       0.0f, 0.0f };

volatile int32_t current_left_count  = 0, last_left_count  = 0;
volatile int32_t current_right_count = 0, last_right_count = 0;

volatile float dL = 0.0f, dR = 0.0f;
volatile float total_left  = 0.0f, total_right  = 0.0f;
volatile float left_speed_mms  = 0.0f;
volatile float right_speed_mms = 0.0f;

volatile float current_phi_rad = 0.0f;
volatile float current_phi_deg = 0.0f;

int PWM_L = 0, PWM_R = 0;

volatile float current_heading_deg = 0.0f;
float gyro_z_offset = 0.0f;

/* ============================================================
   FUNCTION PROTOTYPES
   ============================================================ */
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM5_Init(void);
static void MX_I2C1_Init(void);

void  MPU6050_Calibrate_Gyro_Z(void);
void  update_imu_heading(void);
float constrainf(float x, float mn, float mx);
float pid_compute(PID_t* pid, float setpoint, float measured);
void  pid_reset(PID_t* pid);
void  read_encoders(void);
void  run_motors(void);
void  stop_motors(void);
void  reset_all(void);
float trapezoid_speed(float traveled, float remaining,
                      float spd_max, float spd_min, float ramp_mm);
void  move_distance(float distance_mm, float speed_mms);
void  rotate(float angle_deg, float speed_mms);
void  debug_print(const char* msg);
float get_front_ultrasound(void);
float get_left_ultrasound(void);
float get_right_ultrasound(void);

static float ultrasonic_measure(GPIO_TypeDef* trig_port, uint16_t trig_pin,
                                GPIO_TypeDef* echo_port, uint16_t echo_pin);

/* ============================================================
   MAIN
   ============================================================ */
int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_USART2_UART_Init();
    MX_TIM1_Init();
    MX_TIM2_Init();
    MX_TIM3_Init();
    MX_TIM5_Init();
    MX_I2C1_Init();

    /* Start PWM outputs */
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

    /* Start encoders */
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);

    /* Start µs timer */
    HAL_TIM_Base_Start(&htim5);

    stop_motors();
    HAL_Delay(500);

    /* ---- IMU init & calibrate ---- */
    MPU6050_init();
    HAL_Delay(100);
    MPU6050_Calibrate_Gyro_Z();
    if (fabsf(gyro_z_offset) > 10.0f) gyro_z_offset = 0.0f;

    current_heading_deg = 0.0f;
    imu_initialized = 0;
    update_imu_heading();
    HAL_Delay(500);

    debug_print("Robot Ready — Maze Solver\r\n");

    /* ---- Blue button EXTI (PC13 / EXTI15_10) ---- */
    /* NOTE: NVIC enable is also done in MX_GPIO_Init.
       Re-enabling here ensures correct priority after all inits. */
    HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

    /* ---- Maze init: border walls + initial flood fill ---- */
    maze_init();

    /* ---- Place robot at start corner, then release ---- */
    HAL_Delay(2000);
    debug_print("GO!\r\n");

    /* ---- Hand off to maze runner (infinite loop inside) ---- */
    maze_run();

    /* Should never reach here */
    while (1) {}
}

/* ============================================================
   BLUE BUTTON ISR
   PC13 / EXTI15_10 — falling edge → trigger sprint mode
   ============================================================ */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == B1_Pin)
    {
        maze_trigger_sprint();
    }
}

/* ============================================================
   PID
   ============================================================ */
float pid_compute(PID_t* pid, float setpoint, float measured)
{
    float error      = setpoint - measured;
    float derivative = (error - pid->prev_error) / T_S;

    pid->integral += error * T_S;

    float max_i = (float)PWM_MAX / (pid->ki > 1e-6f ? pid->ki : 1e-6f);
    if (pid->integral >  max_i) pid->integral =  max_i;
    if (pid->integral < -max_i) pid->integral = -max_i;

    pid->prev_error = error;

    return pid->kp * error
         + pid->ki * pid->integral
         + pid->kd * derivative;
}

void pid_reset(PID_t* pid)
{
    pid->integral   = 0.0f;
    pid->prev_error = 0.0f;
}

/* ============================================================
   ENCODERS
   ============================================================ */
void read_encoders(void)
{
    last_left_count  = current_left_count;
    last_right_count = current_right_count;

    current_left_count  = (int32_t)__HAL_TIM_GET_COUNTER(&htim2);
    current_right_count = (int16_t)__HAL_TIM_GET_COUNTER(&htim3);

    int32_t dL_ticks = current_left_count  - last_left_count;
    int32_t dR_ticks = current_right_count - last_right_count;

    dL = (float)dL_ticks * 2.0f * PI_F * LEFT_RADIUS_MM  / LEFT_RESOLUTION;
    dR = (float)dR_ticks * 2.0f * PI_F * RIGHT_RADIUS_MM / RIGHT_RESOLUTION;

    total_left  += dL;
    total_right += dR;

    left_speed_mms  = dL / T_S;
    right_speed_mms = dR / T_S;

    current_phi_rad += (dR - dL) / WHEEL_BASE_MM;
    while (current_phi_rad >  PI_F) current_phi_rad -= 2.0f * PI_F;
    while (current_phi_rad < -PI_F) current_phi_rad += 2.0f * PI_F;
    current_phi_deg = current_phi_rad * 180.0f / PI_F;
}

/* ============================================================
   MOTORS
   ============================================================ */
void run_motors(void)
{
    if (PWM_L >= 0) {
        __HAL_TIM_SET_COMPARE(&htim1, LEFT_RPWM_CH,  0);
        __HAL_TIM_SET_COMPARE(&htim1, LEFT_LPWM_CH,  (uint32_t)PWM_L);
    } else {
        __HAL_TIM_SET_COMPARE(&htim1, LEFT_RPWM_CH,  (uint32_t)(-PWM_L));
        __HAL_TIM_SET_COMPARE(&htim1, LEFT_LPWM_CH,  0);
    }
    if (PWM_R >= 0) {
        __HAL_TIM_SET_COMPARE(&htim1, RIGHT_RPWM_CH, 0);
        __HAL_TIM_SET_COMPARE(&htim1, RIGHT_LPWM_CH, (uint32_t)PWM_R);
    } else {
        __HAL_TIM_SET_COMPARE(&htim1, RIGHT_RPWM_CH, (uint32_t)(-PWM_R));
        __HAL_TIM_SET_COMPARE(&htim1, RIGHT_LPWM_CH, 0);
    }
}

void stop_motors(void)
{
    __HAL_TIM_SET_COMPARE(&htim1, LEFT_RPWM_CH,  0);
    __HAL_TIM_SET_COMPARE(&htim1, LEFT_LPWM_CH,  0);
    __HAL_TIM_SET_COMPARE(&htim1, RIGHT_RPWM_CH, 0);
    __HAL_TIM_SET_COMPARE(&htim1, RIGHT_LPWM_CH, 0);
}

/* ============================================================
   RESET ODOMETRY
   ============================================================ */
void reset_all(void)
{
    __HAL_TIM_SET_COUNTER(&htim2, 0);
    __HAL_TIM_SET_COUNTER(&htim3, 0);
    current_left_count = last_left_count = 0;
    current_right_count = last_right_count = 0;
    total_left = total_right = 0.0f;
    dL = dR = 0.0f;
    left_speed_mms = right_speed_mms = 0.0f;
    pid_reset(&pid_L);
    pid_reset(&pid_R);
    pid_reset(&pid_heading);
}

/* ============================================================
   TRAPEZOIDAL SPEED PROFILE
   ============================================================ */
float trapezoid_speed(float traveled, float remaining,
                      float spd_max, float spd_min, float ramp_mm)
{
    float spd;
    if (traveled < ramp_mm)
        spd = spd_min + (spd_max - spd_min) * (traveled / ramp_mm);
    else if (remaining < ramp_mm)
        spd = spd_min + (spd_max - spd_min) * (remaining / ramp_mm);
    else
        spd = spd_max;
    return constrainf(spd, spd_min, spd_max);
}

/* ============================================================
   MOVE DISTANCE
   ============================================================ */
void move_distance(float distance_mm, float speed_mms)
{
    reset_all();
    current_heading_deg = 0.0f;
    imu_initialized = 0;
    update_imu_heading();
    HAL_Delay(50);
    float target_heading = current_heading_deg;

    float target  = fabsf(distance_mm);
    int   sens    = (distance_mm >= 0) ? 1 : -1;
    float ramp_mm = constrainf(80.0f, 10.0f, target / 2.0f);
    float spd_min = 50.0f;

    while (1)
    {
        read_encoders();
        update_imu_heading();

        float traveled  = fabsf((total_left + total_right) / 2.0f);
        float remaining = target - traveled;
        if (remaining <= 5.0f && traveled > 10.0f) break;

        float spd = trapezoid_speed(traveled, remaining, speed_mms, spd_min, ramp_mm);

        float heading_error      = target_heading - current_heading_deg;
        float heading_correction = constrainf(heading_error * 2.0f, -50.0f, 50.0f);

        float ref_L = spd + (heading_correction * (float)sens);
        float ref_R = spd - (heading_correction * (float)sens);

        float measured_L = (float)sens * left_speed_mms;
        float measured_R = (float)sens * right_speed_mms;

        float out_L = pid_compute(&pid_L, ref_L, measured_L);
        float out_R = pid_compute(&pid_R, ref_R, measured_R);

        PWM_L = (int)constrainf(out_L, 0.0f, (float)PWM_MAX) * sens;
        PWM_R = (int)constrainf(out_R, 0.0f, (float)PWM_MAX) * sens;

        if (PWM_L > 0 && PWM_L < PWM_MIN)  PWM_L = PWM_MIN;
        if (PWM_L < 0 && PWM_L > -PWM_MIN) PWM_L = -PWM_MIN;
        if (PWM_R > 0 && PWM_R < PWM_MIN)  PWM_R = (int)(PWM_MIN * RIGHT_WHEEL_GAIN_FORWARD);
        if (PWM_R < 0 && PWM_R > -PWM_MIN) PWM_R = (int)(-PWM_MIN * RIGHT_WHEEL_GAIN_BACKWARD);

        run_motors();
        HAL_Delay(T_MS);
    }

    stop_motors();
    debug_print("MOVE done\r\n");
}

/* ============================================================
   ROTATE
   ============================================================ */
void rotate(float angle_deg, float speed_mms)
{
    reset_all();

    float initial_heading = current_heading_deg;
    float target_heading  = initial_heading + angle_deg;
    while (target_heading >  180.0f) target_heading -= 360.0f;
    while (target_heading < -180.0f) target_heading += 360.0f;

    uint32_t timeout = 0;

    while (1)
    {
        read_encoders();
        update_imu_heading();

        float angle_error = target_heading - current_heading_deg;
        while (angle_error >  180.0f) angle_error -= 360.0f;
        while (angle_error < -180.0f) angle_error += 360.0f;

        if (fabsf(angle_error) <= 1.5f) break;
        if (++timeout > 2000) break;

        float spd = constrainf(fabsf(angle_error) * 3.5f, 60.0f, speed_mms);
        int   dir = (angle_error > 0) ? 1 : -1;

        float ref_L = -(float)dir * spd;
        float ref_R =  (float)dir * spd;

        float out_L = pid_compute(&pid_L, ref_L, left_speed_mms);
        float out_R = pid_compute(&pid_R, ref_R, right_speed_mms);

        PWM_L = (int)constrainf(out_L, -(float)PWM_MAX, (float)PWM_MAX);
        PWM_R = (int)constrainf(out_R, -(float)PWM_MAX, (float)PWM_MAX);

        if (PWM_L > 0 && PWM_L < PWM_MIN)  PWM_L = PWM_MIN;
        if (PWM_L < 0 && PWM_L > -PWM_MIN) PWM_L = -PWM_MIN;
        if (PWM_R > 0 && PWM_R < PWM_MIN)  PWM_R = (int)(PWM_MIN * RIGHT_WHEEL_GAIN_FORWARD);
        if (PWM_R < 0 && PWM_R > -PWM_MIN) PWM_R = (int)(-PWM_MIN * RIGHT_WHEEL_GAIN_BACKWARD);

        run_motors();
        HAL_Delay(T_MS);
    }

    stop_motors();
    debug_print("ROT done\r\n");
}

/* ============================================================
   UTILITIES
   ============================================================ */
float constrainf(float x, float mn, float mx)
{
    if (x < mn) return mn;
    if (x > mx) return mx;
    return x;
}

void debug_print(const char* msg)
{
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);
}

/* ============================================================
   ULTRASONIC  (HC-SR04 — blocking, µs-timer polled)
   ============================================================ */
static float ultrasonic_measure(GPIO_TypeDef* trig_port, uint16_t trig_pin,
                                GPIO_TypeDef* echo_port, uint16_t echo_pin)
{
    const uint32_t TIMEOUT_US = 30000UL;

    /* Ensure TRIG is low for ≥2 µs */
    HAL_GPIO_WritePin(trig_port, trig_pin, GPIO_PIN_RESET);
    uint32_t t = __HAL_TIM_GET_COUNTER(&htim5);
    while ((__HAL_TIM_GET_COUNTER(&htim5) - t) < 2U);

    /* 10 µs HIGH pulse */
    HAL_GPIO_WritePin(trig_port, trig_pin, GPIO_PIN_SET);
    t = __HAL_TIM_GET_COUNTER(&htim5);
    while ((__HAL_TIM_GET_COUNTER(&htim5) - t) < 10U);
    HAL_GPIO_WritePin(trig_port, trig_pin, GPIO_PIN_RESET);

    /* Wait for ECHO to go HIGH */
    t = __HAL_TIM_GET_COUNTER(&htim5);
    while (HAL_GPIO_ReadPin(echo_port, echo_pin) == GPIO_PIN_RESET) {
        if ((__HAL_TIM_GET_COUNTER(&htim5) - t) > TIMEOUT_US) return -1.0f;
    }

    /* Measure HIGH duration */
    uint32_t start = __HAL_TIM_GET_COUNTER(&htim5);
    while (HAL_GPIO_ReadPin(echo_port, echo_pin) == GPIO_PIN_SET) {
        if ((__HAL_TIM_GET_COUNTER(&htim5) - start) > TIMEOUT_US) return -1.0f;
    }
    uint32_t pulse_us = __HAL_TIM_GET_COUNTER(&htim5) - start;

    /* distance [cm] = pulse_us / 58   →   [mm] = pulse_us / 5.8 */
    return (float)pulse_us / 5.8f;
}

float get_front_ultrasound(void)
{
    return ultrasonic_measure(TRIG_FRONT_GPIO_Port, TRIG_FRONT_Pin,
                              ECHO_FRONT_GPIO_Port, ECHO_FRONT_Pin);
}
float get_left_ultrasound(void)
{
    return ultrasonic_measure(TRIG_LEFT_GPIO_Port, TRIG_LEFT_Pin,
                              ECHO_LEFT_GPIO_Port, ECHO_LEFT_Pin);
}
float get_right_ultrasound(void)
{
    return ultrasonic_measure(TRIG_RIGHT_GPIO_Port, TRIG_RIGHT_Pin,
                              ECHO_RIGHT_GPIO_Port, ECHO_RIGHT_Pin);
}

/* ============================================================
   IMU
   ============================================================ */
void update_imu_heading(void)
{
    float Gx, Gy, Gz;
    MPU6050_Read_Gyro(&Gx, &Gy, &Gz);

    uint32_t now = HAL_GetTick();

    if (!imu_initialized) {
        imu_last_tick   = now;
        imu_initialized = 1;
        return;
    }

    float dt = (float)(now - imu_last_tick) * 0.001f;
    imu_last_tick = now;
    if (dt <= 0.0f || dt > 0.1f) return;

    float gz_dps = Gz - gyro_z_offset;
    if (fabsf(gz_dps) < 0.3f) gz_dps = 0.0f;

    current_heading_deg += gz_dps * dt;
    while (current_heading_deg >  180.0f) current_heading_deg -= 360.0f;
    while (current_heading_deg < -180.0f) current_heading_deg += 360.0f;
}

void MPU6050_Calibrate_Gyro_Z(void)
{
    float    sum     = 0.0f;
    const uint16_t samples = 200;

    for (uint16_t i = 0; i < samples; i++) {
        float Gx, Gy, Gz;
        MPU6050_Read_Gyro(&Gx, &Gy, &Gz);
        sum += Gz;
        HAL_Delay(2);
    }
    gyro_z_offset = sum / (float)samples;
}

/* ============================================================
   CLOCK + PERIPHERAL INIT
   ============================================================ */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState       = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState   = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 8;
    RCC_OscInitStruct.PLL.PLLN = 336;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 2;
    RCC_OscInitStruct.PLL.PLLR = 2;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) Error_Handler();

    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                     | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) Error_Handler();
}

static void MX_I2C1_Init(void)
{
    hi2c1.Instance             = I2C1;
    hi2c1.Init.ClockSpeed      = 400000;
    hi2c1.Init.DutyCycle       = I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1     = 0;
    hi2c1.Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2     = 0;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c1) != HAL_OK) Error_Handler();
}

static void MX_TIM1_Init(void)
{
    TIM_MasterConfigTypeDef        sMasterConfig   = {0};
    TIM_OC_InitTypeDef             sConfigOC       = {0};
    TIM_BreakDeadTimeConfigTypeDef sBreakDeadTime  = {0};

    htim1.Instance               = TIM1;
    htim1.Init.Prescaler         = 0;
    htim1.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim1.Init.Period            = 9999;
    htim1.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.RepetitionCounter = 0;
    htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_PWM_Init(&htim1) != HAL_OK) Error_Handler();

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK) Error_Handler();

    sConfigOC.OCMode       = TIM_OCMODE_PWM1;
    sConfigOC.Pulse        = 0;
    sConfigOC.OCPolarity   = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCNPolarity  = TIM_OCNPOLARITY_HIGH;
    sConfigOC.OCFastMode   = TIM_OCFAST_DISABLE;
    sConfigOC.OCIdleState  = TIM_OCIDLESTATE_RESET;
    sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) Error_Handler();
    if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK) Error_Handler();
    if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK) Error_Handler();
    if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK) Error_Handler();

    sBreakDeadTime.OffStateRunMode  = TIM_OSSR_DISABLE;
    sBreakDeadTime.OffStateIDLEMode = TIM_OSSI_DISABLE;
    sBreakDeadTime.LockLevel        = TIM_LOCKLEVEL_OFF;
    sBreakDeadTime.DeadTime         = 0;
    sBreakDeadTime.BreakState       = TIM_BREAK_DISABLE;
    sBreakDeadTime.BreakPolarity    = TIM_BREAKPOLARITY_HIGH;
    sBreakDeadTime.AutomaticOutput  = TIM_AUTOMATICOUTPUT_DISABLE;
    if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTime) != HAL_OK) Error_Handler();

    HAL_TIM_MspPostInit(&htim1);
}

static void MX_TIM2_Init(void)
{
    TIM_Encoder_InitTypeDef sConfig       = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};

    htim2.Instance               = TIM2;
    htim2.Init.Prescaler         = 0;
    htim2.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim2.Init.Period            = 0xFFFFFFFF;  /* 32-bit full range */
    htim2.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    sConfig.EncoderMode   = TIM_ENCODERMODE_TI12;
    sConfig.IC1Polarity   = TIM_ICPOLARITY_RISING;
    sConfig.IC1Selection  = TIM_ICSELECTION_DIRECTTI;
    sConfig.IC1Prescaler  = TIM_ICPSC_DIV1;
    sConfig.IC1Filter     = 0;
    sConfig.IC2Polarity   = TIM_ICPOLARITY_RISING;
    sConfig.IC2Selection  = TIM_ICSELECTION_DIRECTTI;
    sConfig.IC2Prescaler  = TIM_ICPSC_DIV1;
    sConfig.IC2Filter     = 0;
    if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK) Error_Handler();
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK) Error_Handler();
}

static void MX_TIM3_Init(void)
{
    TIM_Encoder_InitTypeDef sConfig       = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};

    htim3.Instance               = TIM3;
    htim3.Init.Prescaler         = 0;
    htim3.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim3.Init.Period            = 65535;
    htim3.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    sConfig.EncoderMode   = TIM_ENCODERMODE_TI12;
    sConfig.IC1Polarity   = TIM_ICPOLARITY_RISING;
    sConfig.IC1Selection  = TIM_ICSELECTION_DIRECTTI;
    sConfig.IC1Prescaler  = TIM_ICPSC_DIV1;
    sConfig.IC1Filter     = 15;
    sConfig.IC2Polarity   = TIM_ICPOLARITY_RISING;
    sConfig.IC2Selection  = TIM_ICSELECTION_DIRECTTI;
    sConfig.IC2Prescaler  = TIM_ICPSC_DIV1;
    sConfig.IC2Filter     = 0;
    if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK) Error_Handler();
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK) Error_Handler();
}

static void MX_TIM5_Init(void)
{
    /* TIM5 = 32-bit timer, APB1 = 84 MHz → prescaler 167 → 1 tick = 1 µs   */
    /* Period = 0xFFFFFFFF (full 32-bit range, no overflow during 30 ms echo) */
    TIM_ClockConfigTypeDef  sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig      = {0};

    htim5.Instance               = TIM5;
    htim5.Init.Prescaler         = 167;         /* 168 MHz / (167+1) = 1 MHz */
    htim5.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim5.Init.Period            = 0xFFFFFFFF;  /* 32-bit — never overflows   */
    htim5.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_Base_Init(&htim5) != HAL_OK) Error_Handler();

    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK) Error_Handler();

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK) Error_Handler();
}

static void MX_USART2_UART_Init(void)
{
    huart2.Instance          = USART2;
    huart2.Init.BaudRate     = 115200;
    huart2.Init.WordLength   = UART_WORDLENGTH_8B;
    huart2.Init.StopBits     = UART_STOPBITS_1;
    huart2.Init.Parity       = UART_PARITY_NONE;
    huart2.Init.Mode         = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart2) != HAL_OK) Error_Handler();
}

static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* Default output levels */
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, TRIG_FRONT_Pin | TRIG_LEFT_Pin | TRIG_RIGHT_Pin,
                      GPIO_PIN_RESET);

    /* Blue button PC13 — falling-edge EXTI */
    GPIO_InitStruct.Pin  = B1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

    /* LED */
    GPIO_InitStruct.Pin   = LD2_Pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

    /* Ultrasonic TRIG (output) */
    GPIO_InitStruct.Pin   = TRIG_FRONT_Pin | TRIG_LEFT_Pin | TRIG_RIGHT_Pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* Ultrasonic ECHO (input) */
    GPIO_InitStruct.Pin  = ECHO_FRONT_Pin | ECHO_LEFT_Pin | ECHO_RIGHT_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* NVIC for blue button EXTI15_10 */
    HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

/* ============================================================
   ERROR HANDLER
   ============================================================ */
void Error_Handler(void)
{
    __disable_irq();
    while (1) {}
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t* file, uint32_t line) {}
#endif
