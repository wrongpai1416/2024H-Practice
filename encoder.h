#ifndef __ENCODER_H
#define __ENCODER_H

#include "ti_msp_dl_config.h"
#include <stdint.h>

/* 编码器参数 — 根据实际编码器型号修改 */
#define ENCODER_PPR     13      /* 编码器每转脉冲数（霍尔编码器常见 13/11 线） */
#define GEAR_RATIO      30      /* 减速比 */
#define WHEEL_DIA_MM    65      /* 轮子直径 mm */
#define PI_F            3.14159f

/* 脉冲计数（volatile，中断写 / 主循环或定时中断读） */
extern volatile int32_t encoder_left_cnt;
extern volatile int32_t encoder_right_cnt;

void    Encoder_Init(void);
void    Encoder_Update(void);   /* 在 PID 定时中断中调用：读取并清零计数 */

#endif
