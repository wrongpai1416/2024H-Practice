#ifndef __MPU6050_H
#define __MPU6050_H

#include "ti_msp_dl_config.h"
#include <stdint.h>

#define MPU6050_ADDR         0x68
#define MPU6050_PWR_MGMT_1   0x6B
#define MPU6050_SMPLRT_DIV   0x19
#define MPU6050_CONFIG       0x1A
#define MPU6050_GYRO_CONFIG  0x1B
#define MPU6050_ACCEL_CONFIG 0x1C
#define MPU6050_ACCEL_XOUT_H 0x3B
#define MPU6050_GYRO_XOUT_H  0x43
#define MPU6050_WHO_AM_I     0x75

void    MPU6050_Init(void);
void    MPU6050_WriteReg(uint8_t reg, uint8_t data);
uint8_t MPU6050_ReadReg(uint8_t reg);
int16_t MPU6050_ReadGyroZ(void);  /* 读取 Z 轴角速度 */

#endif
