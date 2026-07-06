/*
 * H题自动行驶小车 - 主程序
 *
 * 改下面的 TASK_NUM 选择任务（1/2/3/4），烧录即可运行
 *
 * 场地布局：
 *        A ←——— 直线 ———→ B
 *        |                   |
 *     左半圆              右半圆
 *     (R=40cm)            (R=40cm)
 *        |                   |
 *        C ←——— 直线 ———→ D
 */

#include "ti_msp_dl_config.h"
#include "oled.h"
#include "delay.h"
#include "motor.h"
#include "encoder.h"
#include "ir_sensor.h"
#include "gyro.h"
#include "buzzer.h"
#include "task.h"

/* ========================================
 *  改这里选任务：1 / 2 / 3 / 4
 * ======================================== */
#define TASK_NUM    1

int main(void)
{
    SYSCFG_DL_init();

    /* ---------- 外设初始化 ---------- */
    OLED_Init();
    OLED_Clear();
    OLED_ShowString(0, 0, (uint8_t *)"H-Car Init...", 16);
    OLED_Refresh();

    Encoder_Init();
    Motor_Init();
    Buzzer_Init();
    IR_Init();

    /* MPU6050 + 陀螺仪校准（需要静止约 400ms） */
    Gyro_Init();

    /* 使能 PID 定时中断（10ms） */
    DL_Timer_startCounter(MOTOR_PID_INST);
    NVIC_EnableIRQ(MOTOR_PID_INST_INT_IRQN);

    /* ---------- 显示任务信息 ---------- */
    OLED_Clear();
    OLED_ShowString(0, 0, (uint8_t *)"Task", 16);
    OLED_ShowNum(48, 0, TASK_NUM, 1, 16);
    OLED_ShowString(0, 2, (uint8_t *)"Ready!", 16);
    OLED_Refresh();
    delay_ms(1000);

    /* ---------- 执行任务 ---------- */
    Task_Init(TASK_NUM);

    while (1) {
        Task_Run();

        if (Task_IsFinished()) {
            break;
        }

        delay_ms(10);
    }

    /* ---------- 任务完成 ---------- */
    Motor_Stop();

    OLED_Clear();
    OLED_ShowString(0, 0, (uint8_t *)"Done!", 16);
    OLED_Refresh();

    while (1) {
        /* 任务完成后无限循环 */
    }
}
