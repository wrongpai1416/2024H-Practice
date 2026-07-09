#ifndef __IR_SENSOR_H
#define __IR_SENSOR_H

#include "ti_msp_dl_config.h"
#include <stdint.h>

#define IR_NUM  8

/* 8路循迹传感器引脚定义 — 使用 SysConfig 生成的宏名 */
/* S1=PA18(最右), S2=PB8, S3=PB9, S4=PA23, S5=PA24, S6=PA25, S7=PA26, S8=PA27(最左) */

void IR_Init(void);
void IR_Read(uint8_t *values);
int16_t IR_GetError(uint8_t *values);

#endif
