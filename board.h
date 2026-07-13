#ifndef __BOARD_H__
#define __BOARD_H__

#include "ti_msp_dl_config.h"

#ifndef u8
#define u8 uint8_t
#endif
#ifndef u16
#define u16 uint16_t
#endif
#ifndef u32
#define u32 uint32_t
#endif

/* 延时函数 — 用我们自己的 delay.c */
extern void delay_us(uint32_t us);
extern void delay_ms(uint32_t ms);

/* 空的 printf，避免编译错误 */
#define lc_printf(...)  ((void)0)

#endif
