#ifndef __GYRO_H
#define __GYRO_H

#include <stdint.h>

/* 陀螺仪 yaw 角度（度），volatile 供中断/主循环共享 */
extern volatile float gyro_yaw;

void Gyro_Init(void);
void Gyro_Reset(void);           /* 重置 yaw 为 0 */
void Gyro_Update(float dt);      /* 在定时中断中调用，dt=采样周期(秒) */
float Gyro_GetYaw(void);         /* 获取当前 yaw 角度 */

#endif
