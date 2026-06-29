/* ============================================================
 * MPU6050.h  —  Minimal MPU-6050 driver
 * STM32 HAL I2C  ·  Gyroscope + Accelerometer
 *
 * Uses I2C address 0x68 (AD0 pin = GND).
 * Change MPU6050_ADDR to 0xD2 if AD0 = VCC.
 * ============================================================ */
#ifndef MPU6050_H
#define MPU6050_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

/* ============================================================
   I2C ADDRESS
   AD0 = GND  →  0x68 << 1 = 0xD0
   AD0 = VCC  →  0x69 << 1 = 0xD2
   ============================================================ */
#define MPU6050_ADDR        0xD0

/* ============================================================
   REGISTER MAP  (only what we use)
   ============================================================ */
#define MPU6050_REG_SMPLRT_DIV      0x19
#define MPU6050_REG_CONFIG          0x1A
#define MPU6050_REG_GYRO_CONFIG     0x1B
#define MPU6050_REG_ACCEL_CONFIG    0x1C
#define MPU6050_REG_ACCEL_XOUT_H    0x3B
#define MPU6050_REG_TEMP_OUT_H      0x41
#define MPU6050_REG_GYRO_XOUT_H     0x43
#define MPU6050_REG_PWR_MGMT_1      0x6B
#define MPU6050_REG_WHO_AM_I        0x75

/* ============================================================
   GYRO FULL-SCALE RANGE
   ============================================================ */
typedef enum {
    GYRO_FS_250  = 0x00,   /* ±250  °/s  — 131.0 LSB/(°/s) */
    GYRO_FS_500  = 0x08,   /* ±500  °/s  —  65.5 LSB/(°/s) */
    GYRO_FS_1000 = 0x10,   /* ±1000 °/s  —  32.8 LSB/(°/s) */
    GYRO_FS_2000 = 0x18    /* ±2000 °/s  —  16.4 LSB/(°/s) */
} Gyro_FS_t;

/* ============================================================
   ACCEL FULL-SCALE RANGE
   ============================================================ */
typedef enum {
    ACCEL_FS_2G  = 0x00,   /* ±2 g  — 16384 LSB/g */
    ACCEL_FS_4G  = 0x08,   /* ±4 g  —  8192 LSB/g */
    ACCEL_FS_8G  = 0x10,   /* ±8 g  —  4096 LSB/g */
    ACCEL_FS_16G = 0x18    /* ±16 g —  2048 LSB/g */
} Accel_FS_t;

/* ============================================================
   PUBLIC API
   ============================================================ */

/**
 * MPU6050_init()
 *   Wake the device, set clock source, configure gyro/accel ranges.
 *   Must be called after MX_I2C1_Init() and a short HAL_Delay(100).
 */
void MPU6050_init(void);

/**
 * MPU6050_Read_Gyro()
 *   Read calibrated gyroscope data in degrees/second.
 *   @param Gx  pointer — roll  rate [°/s]
 *   @param Gy  pointer — pitch rate [°/s]
 *   @param Gz  pointer — yaw   rate [°/s]  ← used for heading
 */
void MPU6050_Read_Gyro(float* Gx, float* Gy, float* Gz);

/**
 * MPU6050_Read_Accel()
 *   Read accelerometer data in g.
 *   @param Ax  pointer — X acceleration [g]
 *   @param Ay  pointer — Y acceleration [g]
 *   @param Az  pointer — Z acceleration [g]
 */
void MPU6050_Read_Accel(float* Ax, float* Ay, float* Az);

/**
 * MPU6050_Read_Temp()
 *   Read die temperature.
 *   @return temperature in °C
 */
float MPU6050_Read_Temp(void);

/**
 * MPU6050_WhoAmI()
 *   Read the WHO_AM_I register.  Expected value: 0x68.
 *   Useful for verifying I2C connectivity.
 *   @return device ID byte
 */
uint8_t MPU6050_WhoAmI(void);

#ifdef __cplusplus
}
#endif

#endif /* MPU6050_H */
