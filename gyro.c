#include "gyro.h"
#include "mpu6050.h"
#include "encoder.h"
#include "delay.h"

/* ========================================
 *  陀螺仪开关：0=未接（跳过I2C），1=已接
 * ======================================== */
#define GYRO_ENABLE   1

/* 陀螺仪 Z 轴零偏（开机校准 + 运行时自动修正） */
static float gyro_offset = 0.0f;

/* 累计 yaw 角度（度） */
volatile float gyro_yaw = 0.0f;

/* 校准采样数 */
#define CALIB_SAMPLES  500

/* 自动漂移补偿：编码器同速时修正零偏 */
static uint8_t  auto_cal_enabled = 0;       /* 使能开关 */
static int32_t  prev_enc_left  = 0;
static int32_t  prev_enc_right = 0;
static float    prev_yaw       = 0.0f;

void Gyro_Init(void)
{
#if GYRO_ENABLE
    MPU6050_Init();
    delay_ms(500);  /* 等 MPU6050 温度稳定 */

    /* 零偏校准：静止状态采样 500 次取平均 */
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
    auto_cal_enabled = 0;
}

void Gyro_Reset(void)
{
    gyro_yaw = 0.0f;
}

/* 使能运行时自动漂移补偿（起步稳定后调用） */
void Gyro_EnableAutoCal(void)
{
    prev_enc_left  = encoder_left_cnt;
    prev_enc_right = encoder_right_cnt;
    prev_yaw       = gyro_yaw;
    auto_cal_enabled = 1;
}

void Gyro_Update(float dt)
{
#if GYRO_ENABLE
    int16_t raw = MPU6050_ReadGyroZ();
    float omega = ((float)raw - gyro_offset) / 65.5f;
    gyro_yaw += omega * dt;

    /* ---- 运行时自动漂移补偿 ---- */
    if (auto_cal_enabled) {
        int32_t dl = encoder_left_cnt  - prev_enc_left;
        int32_t dr = encoder_right_cnt - prev_enc_right;
        prev_enc_left  = encoder_left_cnt;
        prev_enc_right = encoder_right_cnt;

        int32_t diff = dl - dr;
        if (diff < 0) diff = -diff;

        /* 两轮同速（差值<3脉冲）= 直走 → yaw不该变 */
        if (diff < 3) {
            float yaw_change = gyro_yaw - prev_yaw;
            if (yaw_change > 0.2f || yaw_change < -0.2f) {
                /* yaw变了但车在直走 → 是漂移，修正零偏 */
                /* yaw_change > 0 说明零偏偏小，需要加大 */
                gyro_offset += yaw_change / dt * 0.01f;
            }
        }
        prev_yaw = gyro_yaw;
    }
#else
    (void)dt;
#endif
}

float Gyro_GetYaw(void)
{
    return gyro_yaw;
}
