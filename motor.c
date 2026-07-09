#include "motor.h"
#include "encoder.h"
#include "pid.h"
#include "gyro.h"

/* 10ms tick 计数（main.c 用它算时间） */
volatile uint32_t tick_10ms = 0;

/* ---------- 两个电机的 PID 控制器 ---------- */
static PID_Controller pid_left;
static PID_Controller pid_right;

/* 目标速度（脉冲数 / 10ms），正=前进，负=后退 */
static volatile float target_left  = 0;
static volatile float target_right = 0;

/* 当前速度（脉冲数 / 10ms） */
static volatile float speed_left  = 0;
static volatile float speed_right = 0;

/* ---------- 基础 PWM 设置 ---------- */
static void Motor_SetPWM(uint8_t ch, int16_t duty)
{
    if (duty < 0) duty = -duty;
    if (duty > 1000) duty = 1000;
    DL_TimerG_setCaptureCompareValue(PWMAB_INST, (uint32_t)duty,
                                     ch == 0 ? DL_TIMER_CC_0_INDEX
                                             : DL_TIMER_CC_1_INDEX);
}

/* speed: -1000 ~ +1000, 正=前进, 负=后退 */
void Motor_SetLeft(int16_t speed)
{
    if (speed > 0) {
        DL_GPIO_setPins(MOTOR_AIN1_PORT, MOTOR_AIN1_PIN);
        DL_GPIO_clearPins(MOTOR_AIN2_PORT, MOTOR_AIN2_PIN);
    } else if (speed < 0) {
        DL_GPIO_clearPins(MOTOR_AIN1_PORT, MOTOR_AIN1_PIN);
        DL_GPIO_setPins(MOTOR_AIN2_PORT, MOTOR_AIN2_PIN);
    } else {
        DL_GPIO_clearPins(MOTOR_AIN1_PORT, MOTOR_AIN1_PIN);
        DL_GPIO_clearPins(MOTOR_AIN2_PORT, MOTOR_AIN2_PIN);
    }
    Motor_SetPWM(0, speed);
}

void Motor_SetRight(int16_t speed)
{
    if (speed > 0) {
        DL_GPIO_setPins(MOTOR_BIN1_PORT, MOTOR_BIN1_PIN);
        DL_GPIO_clearPins(MOTOR_BIN2_PORT, MOTOR_BIN2_PIN);
    } else if (speed < 0) {
        DL_GPIO_clearPins(MOTOR_BIN1_PORT, MOTOR_BIN1_PIN);
        DL_GPIO_setPins(MOTOR_BIN2_PORT, MOTOR_BIN2_PIN);
    } else {
        DL_GPIO_clearPins(MOTOR_BIN1_PORT, MOTOR_BIN1_PIN);
        DL_GPIO_clearPins(MOTOR_BIN2_PORT, MOTOR_BIN2_PIN);
    }
    Motor_SetPWM(1, speed);
}

void Motor_Stop(void)
{
    DL_GPIO_clearPins(MOTOR_AIN1_PORT, MOTOR_AIN1_PIN);
    DL_GPIO_clearPins(MOTOR_AIN2_PORT, MOTOR_AIN2_PIN);
    DL_GPIO_clearPins(MOTOR_BIN1_PORT, MOTOR_BIN1_PIN);
    DL_GPIO_clearPins(MOTOR_BIN2_PORT, MOTOR_BIN2_PIN);
    DL_TimerG_setCaptureCompareValue(PWMAB_INST, 0, DL_TIMER_CC_0_INDEX);
    DL_TimerG_setCaptureCompareValue(PWMAB_INST, 0, DL_TIMER_CC_1_INDEX);
}

/* ---------- 初始化（含 PID） ---------- */
void Motor_Init(void)
{
    DL_GPIO_setPins(MOTOR_STBY_PORT, MOTOR_STBY_PIN);
    Motor_Stop();
    DL_TimerG_startCounter(PWMAB_INST);

    /* 电机速度 PID（保持原来能转的参数） */
    PID_Init(&pid_left,  20.0f, 0.8f, 0.3f, 1000.0f, 500.0f);
    PID_Init(&pid_right, 20.0f, 0.8f, 0.3f, 1000.0f, 500.0f);
}

/* ---------- 设置目标速度（外部调用） ---------- */
void Motor_SetTarget(float left, float right)
{
    target_left  = left;
    target_right = right;
}

/* 注意：定时中断处理已移至 task.c 的 MOTOR_PID_INST_IRQHandler */
