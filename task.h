#ifndef __TASK_H
#define __TASK_H

#include <stdint.h>

/* 任务状态机 */
typedef enum {
    STATE_IDLE,           /* 空闲 */
    STATE_LINE_FOLLOW,    /* 直线循迹 */
    STATE_SEMI_TURN,      /* 半圆弧转向（陀螺仪控制） */
    STATE_ARRIVED,        /* 到达点位（声光提示） */
    STATE_STOP            /* 停车 */
} TaskState;

/* 半圆弧方向 */
typedef enum {
    TURN_NONE,
    TURN_LEFT,            /* 左半圆（逆时针，yaw 减小） */
    TURN_RIGHT            /* 右半圆（顺时针，yaw 增大） */
} TurnDir;

void Task_Init(uint8_t task_num);
void Task_Run(void);          /* 在主循环中调用，执行一步状态机 */
uint8_t Task_IsFinished(void); /* 任务是否完成 */

#endif
