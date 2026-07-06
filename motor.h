#ifndef __MOTOR_H
#define __MOTOR_H

#include "ti_msp_dl_config.h"
#include <stdint.h>

/* TB6612 方向控制 — 使用 SysConfig 生成的宏名 */
#define MOTOR_AIN1_PORT   DC_MOTOR_AIN1_PORT
#define MOTOR_AIN1_PIN    DC_MOTOR_AIN1_PIN   /* PA8 */
#define MOTOR_AIN2_PORT   DC_MOTOR_AIN2_PORT
#define MOTOR_AIN2_PIN    DC_MOTOR_AIN2_PIN   /* PA9 */
#define MOTOR_BIN1_PORT   DC_MOTOR_BIN1_PORT
#define MOTOR_BIN1_PIN    DC_MOTOR_BIN1_PIN   /* PA0 */
#define MOTOR_BIN2_PORT   DC_MOTOR_BIN2_PORT
#define MOTOR_BIN2_PIN    DC_MOTOR_BIN2_PIN   /* PA1 */
#define MOTOR_STBY_PORT   DC_MOTOR_STBY_PORT
#define MOTOR_STBY_PIN    DC_MOTOR_STBY_PIN   /* PB24 */

/* 编码器引脚 */
#define ENCODER_RA_PORT   DC_MOTOR_RA_PORT
#define ENCODER_RA_PIN    DC_MOTOR_RA_PIN     /* PA21 — 右A(中断) */
#define ENCODER_RB_PORT   DC_MOTOR_RB_PORT
#define ENCODER_RB_PIN    DC_MOTOR_RB_PIN     /* PA22 — 右B(读方向) */
#define ENCODER_LA_PORT   DC_MOTOR_LA_PORT
#define ENCODER_LA_PIN    DC_MOTOR_LA_PIN     /* PB19 — 左A(中断) */
#define ENCODER_LB_PORT   DC_MOTOR_LB_PORT
#define ENCODER_LB_PIN    DC_MOTOR_LB_PIN     /* PB20 — 左B(读方向) */

void Motor_Init(void);
void Motor_SetLeft(int16_t speed);   /* -1000 ~ +1000 */
void Motor_SetRight(int16_t speed);
void Motor_Stop(void);
void Motor_SetTarget(float left, float right);  /* 设置 PID 目标速度（脉冲/10ms） */
void Motor_PID_Update(void);                     /* 10ms PID 中断中调用 */

#endif
