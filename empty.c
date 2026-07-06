/*
 * 5路灰度循迹 + 编码器PID（分段修正 + 丢线低速直走）
 */

#include "ti_msp_dl_config.h"
#include "oled.h"
#include "delay.h"
#include "motor.h"
#include "encoder.h"
#include "ir_sensor.h"

/* 循迹基础速度（脉冲/10ms） */
#define BASE_SPEED      25.0f
/* 丢线时低速直走 */
#define LOST_SPEED      10.0f

int main(void)
{
    SYSCFG_DL_init();

    OLED_Init();
    OLED_Clear();
    OLED_ShowString(0, 0, (uint8_t *)"Line Follow", 16);
    OLED_Refresh();

    Encoder_Init();
    Motor_Init();

    /* 使能 PID 定时中断 */
    DL_Timer_startCounter(MOTOR_PID_INST);
    NVIC_EnableIRQ(MOTOR_PID_INST_INT_IRQN);

    while (1) {
        uint8_t vals[IR_NUM];
        IR_Read(vals);
        int16_t error = IR_GetError(vals);

        /* 5个全白 → 丢线了，低速直走 */
        if (vals[0] == 0 && vals[1] == 0 && vals[2] == 0 &&
            vals[3] == 0 && vals[4] == 0) {
            Motor_SetTarget(LOST_SPEED, LOST_SPEED);
        }
        /* 有线 → 分段修正循迹 */
        else {
            float abs_err = (float)(error > 0 ? error : -error);
            float correction;

            if (abs_err < 100.0f) {
                correction = abs_err * 0.02f;
            } else if (abs_err < 250.0f) {
                correction = abs_err * 0.07f;
            } else {
                correction = abs_err * 0.15f;
            }

            if (error > 0) {
                Motor_SetTarget(BASE_SPEED + correction,
                                BASE_SPEED - correction);
            } else {
                Motor_SetTarget(BASE_SPEED - correction,
                                BASE_SPEED + correction);
            }
        }

        delay_ms(10);
    }
}
