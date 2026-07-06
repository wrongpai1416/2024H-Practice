#ifndef __BUZZER_H
#define __BUZZER_H

#include "ti_msp_dl_config.h"
#include <stdint.h>

/* LED 引脚直接使用 SysConfig 生成的宏 */
/* LED_PORT = GPIOA, LED_D0_PIN = DL_GPIO_PIN_14 (PA14) */

void Buzzer_Init(void);
void Buzzer_Beep(uint16_t ms);    /* LED 闪烁 ms 毫秒 */
void Buzzer_ShortBeep(void);      /* 短促提示 200ms */

#endif
