#include "gyro.h"
#include "mpu6050.h"
#include "delay.h"

/* ========================================
 *  陀螺仪开关：0=未接（跳过I2C），1=已接
 * ======================================== */
#define GYRO_ENABLE   1

/* 陀螺仪 Z 轴零偏（开机校准） */
static float gyro_offset = 0.0f;

/* 累计 yaw 角度（度） */
volatile float gyro_yaw = 0.0f;

/* 校准采样数 */
#define CALIB_SAMPLES  200

void Gyro_Init(void)
{
#if GYRO_ENABLE
    MPU6050_Init();

    /* 零偏校准：静止状态采样 200 次取平均 */
    int32_t sum = 0;
    for (int i = 0; i < CALIB_SAMPLES; i++) {
        sum += MPU6050_ReadGyroZ();
        delay_ms(2);
    }
    gyro_offset = (float)sum / CALIB_SAMPLES;
#else
    gyro_offset = 0.0f;
#endif
    gyro_yaw = 0.0f;
}

void Gyro_Reset(void)
{
    gyro_yaw = 0.0f;
}

void Gyro_Update(float dt)
{
#if GYRO_ENABLE
    int16_t raw = MPU6050_ReadGyroZ();
    float omega = ((float)raw - gyro_offset) / 65.5f;
    gyro_yaw += omega * dt;
#else
    (void)dt;
#endif
}

float Gyro_GetYaw(void)
{
    return gyro_yaw;
}
