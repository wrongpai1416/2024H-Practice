/*
 * H题自动行驶小车 - 主程序
 *
 * =============================================
 *  改这里选任务：1 / 2 / 3 / 4
 * =============================================
 *  TASK_NUM 在下面第 78 行
 *
 * =============================================
 *  可调参数速查（都在 task.c 里）
 * =============================================
 *
 *  【电机速度 PID】 → motor.c 第 80~81 行
 *    PID_Init(&pid_left,  Kp, Ki, Kd, max_out, max_int)
 *    - 控制电机转速，太小车不动，太大车抖
 *    - 当前值：Kp=20, Ki=0.8, Kd=0.3
 *
 *  【直线基础速度】 → task.c 第 30 行
 *    #define BASE_SPEED  20.0f
 *    - 目标速度（脉冲/10ms），越大越快
 *
 *  【直线走多远】 → task.c 第 56~57 行
 *    #define DIST_AB_CM  20.0f   ← A到B距离(cm)
 *    #define DIST_CD_CM  20.0f   ← C到D距离(cm)
 *    - 距离÷PULSES_PER_CM = 编码器脉冲数
 *
 *  【编码器脉冲/厘米】 → task.c 第 43 行
 *    #define PULSES_PER_CM  100.0f
 *    - 实测：让车走1米看编码器读数，除以100
 *
 *  【方向修正（直走不偏）】 → task.c 第 47~53 行
 *    编码器差速PID：
 *      ENC_DIFF_KP  = 0.35  ← 左右轮差速P，太小偏，太大抖
 *      ENC_DIFF_KI  = 0.03  ← 消除稳态误差
 *      ENC_DIFF_KD  = 0.15  ← 抑制超调
 *      ENC_DIFF_MAX_I = 3.0 ← I项限幅
 *    陀螺仪航向PD：
 *      GYRO_KP = 1.2  ← 航向P，右偏加大，左偏减小
 *      GYRO_KD = 0.4  ← 航向D，抑制震荡
 *    总修正限幅：
 *      STEERING_MAX = 5.0  ← 最大修正量
 *
 *  【半圆弧转向】 → task.c 第 33~36 行
 *    TURN_SPEED = 18.0f       ← 转弯速度
 *    TURN_TARGET_ANGLE = 175  ← 目标转角(度)，不到180做补偿
 *    TURN_KP = 0.8            ← 转向P
 *    TURN_KD = 0.3            ← 转向D
 *
 *  【超时保底】 → task.c 第 44 行
 *    MAX_STRAIGHT_MS = 15000  ← 直线段最大15秒
 *
 * =============================================
 *  调参口诀
 * =============================================
 *  右偏 → GYRO_KP 加大
 *  左偏 → GYRO_KP 减小
 *  抖动 → GYRO_KD 加大 或 GYRO_KP 减小
 *  走不远/太远 → 调 DIST_AB_CM
 *  速度太快/太慢 → 调 BASE_SPEED
 *  转弯不到位 → TURN_TARGET_ANGLE 加大
 *  转弯冲出线 → TURN_SPEED 减小
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

extern volatile uint32_t tick_10ms;

int main(void)
{
    SYSCFG_DL_init();

    OLED_Init();
    OLED_Clear();
    OLED_ShowString(0, 0, (uint8_t *)"Task", 16);
    OLED_ShowNum(48, 0, TASK_NUM, 1, 16);
    OLED_Refresh();

    Encoder_Init();
    Motor_Init();
    Buzzer_Init();
    IR_Init();
    Gyro_Init();

    Task_Init(TASK_NUM);
    tick_10ms = 0;
    DL_Timer_startCounter(MOTOR_PID_INST);
    NVIC_EnableIRQ(MOTOR_PID_INST_INT_IRQN);

    while (1) {
        Task_Run();
        if (Task_IsFinished()) break;
        delay_ms(10);
    }

    uint32_t total_ms = tick_10ms * 10;
    NVIC_DisableIRQ(MOTOR_PID_INST_INT_IRQN);
    Motor_Stop();

    OLED_Clear();
    OLED_ShowString(0, 0, (uint8_t *)"Done!", 16);
    OLED_ShowNum(0, 16, total_ms / 1000, 2, 16);
    OLED_ShowString(24, 16, (uint8_t *)".", 16);
    OLED_ShowNum(36, 16, (total_ms % 1000) / 10, 2, 16);
    OLED_ShowString(60, 16, (uint8_t *)"s", 16);
    OLED_Refresh();

    while (1) {}
}
