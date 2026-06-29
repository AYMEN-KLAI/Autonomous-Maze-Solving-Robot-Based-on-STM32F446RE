/* ============================================================
 * MPU6050.c  —  Minimal MPU-6050 driver
 * STM32 HAL I2C  ·  Gyroscope + Accelerometer
 * ============================================================ */

#include "MPU6050.h"

/* ============================================================
   EXTERNAL I2C HANDLE  (defined in main.c)
   ============================================================ */
extern I2C_HandleTypeDef hi2c1;

/* ============================================================
   SENSITIVITY CONSTANTS
   Set to match the chosen full-scale range below.
   GYRO_FS_500  →  65.5  LSB/(°/s)
   ACCEL_FS_2G  →  16384 LSB/g
   ============================================================ */
#define GYRO_SENSITIVITY    65.5f    /* LSB per °/s  — FS=±500 °/s  */
#define ACCEL_SENSITIVITY   16384.0f /* LSB per g    — FS=±2 g       */

/* ============================================================
   LOW-LEVEL HELPERS
   ============================================================ */

static void mpu_write(uint8_t reg, uint8_t data)
{
    uint8_t buf[2] = { reg, data };
    HAL_I2C_Master_Transmit(&hi2c1, MPU6050_ADDR, buf, 2, 10);
}

static void mpu_read(uint8_t reg, uint8_t* dst, uint16_t len)
{
    HAL_I2C_Master_Transmit(&hi2c1, MPU6050_ADDR, &reg, 1, 10);
    HAL_I2C_Master_Receive (&hi2c1, MPU6050_ADDR, dst, len, 20);
}

static inline int16_t combine(uint8_t hi, uint8_t lo)
{
    return (int16_t)((uint16_t)hi << 8 | lo);
}

/* ============================================================
   INIT
   ============================================================ */
void MPU6050_init(void)
{
    /* 1. Wake up — clear SLEEP bit, use internal 8 MHz oscillator */
    mpu_write(MPU6050_REG_PWR_MGMT_1, 0x00);
    HAL_Delay(10);

    /* 2. Use PLL with X-axis gyroscope reference (more stable) */
    mpu_write(MPU6050_REG_PWR_MGMT_1, 0x01);
    HAL_Delay(10);

    /* 3. Sample rate divider: SMPLRT_DIV = 7  → 1 kHz / (1+7) = 125 Hz */
    mpu_write(MPU6050_REG_SMPLRT_DIV, 0x07);

    /* 4. DLPF config: bandwidth ~44 Hz (config=3), reduces gyro noise */
    mpu_write(MPU6050_REG_CONFIG, 0x03);

    /* 5. Gyro full-scale: ±500 °/s  (GYRO_FS_500) */
    mpu_write(MPU6050_REG_GYRO_CONFIG, (uint8_t)GYRO_FS_500);

    /* 6. Accel full-scale: ±2 g  (ACCEL_FS_2G) */
    mpu_write(MPU6050_REG_ACCEL_CONFIG, (uint8_t)ACCEL_FS_2G);

    HAL_Delay(20);
}

/* ============================================================
   READ GYROSCOPE   [°/s]
   ============================================================ */
void MPU6050_Read_Gyro(float* Gx, float* Gy, float* Gz)
{
    uint8_t buf[6];
    mpu_read(MPU6050_REG_GYRO_XOUT_H, buf, 6);

    *Gx = (float)combine(buf[0], buf[1]) / GYRO_SENSITIVITY;
    *Gy = (float)combine(buf[2], buf[3]) / GYRO_SENSITIVITY;
    *Gz = (float)combine(buf[4], buf[5]) / GYRO_SENSITIVITY;
}

/* ============================================================
   READ ACCELEROMETER   [g]
   ============================================================ */
void MPU6050_Read_Accel(float* Ax, float* Ay, float* Az)
{
    uint8_t buf[6];
    mpu_read(MPU6050_REG_ACCEL_XOUT_H, buf, 6);

    *Ax = (float)combine(buf[0], buf[1]) / ACCEL_SENSITIVITY;
    *Ay = (float)combine(buf[2], buf[3]) / ACCEL_SENSITIVITY;
    *Az = (float)combine(buf[4], buf[5]) / ACCEL_SENSITIVITY;
}

/* ============================================================
   READ TEMPERATURE   [°C]
   ============================================================ */
float MPU6050_Read_Temp(void)
{
    uint8_t buf[2];
    mpu_read(MPU6050_REG_TEMP_OUT_H, buf, 2);
    int16_t raw = combine(buf[0], buf[1]);
    return (float)raw / 340.0f + 36.53f;
}

/* ============================================================
   WHO AM I
   ============================================================ */
uint8_t MPU6050_WhoAmI(void)
{
    uint8_t id = 0;
    mpu_read(MPU6050_REG_WHO_AM_I, &id, 1);
    return id;
}
