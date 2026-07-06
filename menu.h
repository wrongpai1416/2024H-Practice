#ifndef __MENU_H
#define __MENU_H

#include <stdint.h>

/* 任务编号 */
#define TASK_1_AB       1   /* A→B 直线 */
#define TASK_2_LOOP1    2   /* A→B→C→D→A */
#define TASK_3_LOOP2    3   /* A→C→B→D→A */
#define TASK_4_LAPS     4   /* 任务3路径 ×4圈 */

/* 按键状态（中断置位，主循环清除） */
extern volatile uint8_t key_up_pressed;
extern volatile uint8_t key_down_pressed;

void    Menu_Init(void);
uint8_t Menu_SelectTask(void);   /* 显示菜单，返回选择的任务编号 1~4 */

#endif
