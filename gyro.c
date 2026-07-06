#include "gyro.h"
#include "mpu6050.h"

/* 陀螺仪 Z 轴零偏（开机校准） */
static float gyro_offset = 0.0f;

/* 累计 yaw 角度（度） */
volatile float gyro_yaw = 0.0f;

/* 校准采样数 */
#define CALIB_SAMPLES  200

void Gyro_Init(void)
{
    MPU6050_Init();

    /* 零偏校准：静止状态采样 200 次取平均 */
    int32_t sum = 0;
    for (int i = 0; i < CALIB_SAMPLES; i++) {
        sum += MPU6050_ReadGyroZ();
        delay_ms(2);
    }
    gyro_offset = (float)sum / CALIB_SAMPLES;

    gyro_yaw = 0.0f;
}

void Gyro_Reset(void)
{
    gyro_yaw = 0.0f;
}

void Gyro_Update(float dt)
{
    int16_t raw = MPU6050_ReadGyroZ();
    /* 减去零偏，转换为 °/s（±500°/s 模式，灵敏度 65.5） */
    float omega = ((float)raw - gyro_offset) / 65.5f;

    /* 积分得到角度 */
    gyro_yaw += omega * dt;
}

float Gyro_GetYaw(void)
{
    return gyro_yaw;
}
