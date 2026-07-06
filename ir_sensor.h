#ifndef __IR_SENSOR_H
#define __IR_SENSOR_H

#include "ti_msp_dl_config.h"
#include <stdint.h>

#define IR_NUM  5

/* 5路循迹传感器引脚定义 — 使用 SysConfig 生成的宏名 */
#define IR_L2_PORT   HUIDU_L2_PORT
#define IR_L2_PIN    HUIDU_L2_PIN    /* PA17 */
#define IR_L1_PORT   HUIDU_L1_PORT
#define IR_L1_PIN    HUIDU_L1_PIN    /* PB8 */
#define IR_M_PORT    HUIDU_M_PORT
#define IR_M_PIN     HUIDU_M_PIN     /* PB9 */
#define IR_R1_PORT   HUIDU_R1_PORT
#define IR_R1_PIN    HUIDU_R1_PIN    /* PA24 */
#define IR_R2_PORT   HUIDU_R2_PORT
#define IR_R2_PIN    HUIDU_R2_PIN    /* PA2 */

void IR_Init(void);
void IR_Read(uint8_t *values);
int16_t IR_GetError(uint8_t *values);

#endif
