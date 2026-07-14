/*
 * H题自动行驶小车 - 基于 Jamie 答案的控制逻辑
 *
 * 控制方式：位置式 PID 三层串级
 *   里程 PID → 基础速度
 *   角度 PID → 航向修正（陀螺仪）
 *   速度 PID → 左右轮独立闭环
 */

#include "task.h"
#include "motor.h"
#include "ir_sensor.h"
#include "gyro.h"
#include "pid.h"
#include "buzzer.h"
#include "oled.h"
#include "delay.h"
#include "encoder.h"
#include <math.h>

/* ---------- 速度参数 ---------- */
#define TRACK_SPEED     3.0f        /* 循迹基础速度 */
#define PWM_LIMIT       2000.0f     /* PWM 限幅 */

/* ---------- 循迹 PID ---------- */
#define TRACK_KP        14.0f

/* ---------- 里程 PID ---------- */
#define MILEAGE_KP      4.0f
#define MILEAGE_KI      0.1f

/* ---------- 直行角度 PID（Jamie 参数） ---------- */
#define STRAIGHT_KP     0.22f
#define STRAIGHT_KD     0.91f

/* ---------- 速度 PID ---------- */
#define SPEED_KP        10.0f
#define SPEED_KI        5.0f
#define SPEED_KD        3.0f

/* ---------- 里程参数 ---------- */
#define DIST_AB_PULSES  1600        /* A→B 脉冲数（需实测） */

/* ---------- 分段判定 ---------- */
#define Black_CNT       30
#define White_CNT       1000

/* ---------- flag 定义 ---------- */
static volatile int flag = 0;
static volatile int flag_en = 0;
static volatile uint8_t finished = 0;
volatile uint8_t current_task = 0;

/* PID 实例 */
static PID_Controller speed_pid[2];     /* 左右轮速度 */
static PID_Controller mileage_pid;      /* 里程 */
static PID_Controller straight_pid;     /* 航向 */
static PID_Controller track_pid;        /* 循迹 */

/* 陀螺仪目标角度 */
static float target_yaw = 0;

/* 编码器速度 */
static float actual_speed[2] = {0, 0};

/* ---------- 限幅 ---------- */
static float clamp(float val, float min, float max)
{
    if (val > max) return max;
    if (val < min) return min;
    return val;
}

/* ---------- 设置电机 PWM ---------- */
static void Set_Pwm(int left, int right)
{
    left  = (int)clamp((float)left,  -PWM_LIMIT, PWM_LIMIT);
    right = (int)clamp((float)right, -PWM_LIMIT, PWM_LIMIT);

    if (left > 0) {
        DL_GPIO_setPins(MOTOR_AIN1_PORT, MOTOR_AIN1_PIN);
        DL_GPIO_clearPins(MOTOR_AIN2_PORT, MOTOR_AIN2_PIN);
    } else {
        DL_GPIO_clearPins(MOTOR_AIN1_PORT, MOTOR_AIN1_PIN);
        DL_GPIO_setPins(MOTOR_AIN2_PORT, MOTOR_AIN2_PIN);
    }
    DL_TimerG_setCaptureCompareValue(PWMAB_INST, (uint32_t)(left > 0 ? left : -left),
                                     DL_TIMER_CC_0_INDEX);

    if (right > 0) {
        DL_GPIO_setPins(MOTOR_BIN1_PORT, MOTOR_BIN1_PIN);
        DL_GPIO_clearPins(MOTOR_BIN2_PORT, MOTOR_BIN2_PIN);
    } else {
        DL_GPIO_clearPins(MOTOR_BIN1_PORT, MOTOR_BIN1_PIN);
        DL_GPIO_setPins(MOTOR_BIN2_PORT, MOTOR_BIN2_PIN);
    }
    DL_TimerG_setCaptureCompareValue(PWMAB_INST, (uint32_t)(right > 0 ? right : -right),
                                     DL_TIMER_CC_1_INDEX);
}

/* ---------- 读取编码器速度（10ms 调用一次） ---------- */
static void update_encoder(void)
{
    static int32_t last_left = 0, last_right = 0;
    int32_t cur_left  = encoder_left_cnt;
    int32_t cur_right = encoder_right_cnt;
    actual_speed[0] = (float)(cur_left  - last_left);
    actual_speed[1] = (float)(cur_right - last_right);
    last_left  = cur_left;
    last_right = cur_right;
}

/* ---------- 角度误差（处理 ±180 跨越） ---------- */
static float calc_angle_error(float target, float current)
{
    float error = target - current;
    if (error >  180.0f) error -= 360.0f;
    if (error < -180.0f) error += 360.0f;
    return error;
}

/* ---------- 8路循迹加权求和 ---------- */
static int Incremental_Quantity(void)
{
    uint8_t v[IR_NUM];
    IR_Read(v);
    int value = 0;
    if (v[0]) value += 12;
    if (v[1]) value += 9;
    if (v[2]) value += 7;
    if (v[3]) value += 3;
    if (v[4]) value -= 3;
    if (v[5]) value -= 7;
    if (v[6]) value -= 9;
    if (v[7]) value -= 12;
    return value;
}

/* ---------- 分段判断 ---------- */
static void Follow_Route(void)
{
    static int cnt = 0;
    uint8_t v[IR_NUM];

    if (flag == 1) {
        if (encoder_left_cnt >= DIST_AB_PULSES) {
            flag = 0;
            finished = 1;
        }
    }
    else if (flag == 2) {
        IR_Read(v);
        if (!(v[0] || v[1] || v[2] || v[3] || v[4] || v[5] || v[6] || v[7])) {
            cnt++;
            if (cnt > White_CNT) { flag = 3; cnt = 0; }
        } else { cnt = 0; }
    }
    else if (flag == 3) {
        if (encoder_left_cnt >= DIST_AB_PULSES * 2) {
            flag = 4;
        }
    }
    else if (flag == 4) {
        IR_Read(v);
        if (!(v[0] || v[1] || v[2] || v[3] || v[4] || v[5] || v[6] || v[7])) {
            cnt++;
            if (cnt > White_CNT) { flag = 0; finished = 1; cnt = 0; }
        } else { cnt = 0; }
    }
}

/* ========== Task5 纯循迹测试（独立，不走状态机） ========== */
static void Control_Task5(void)
{
    uint8_t v[IR_NUM];
    IR_Read(v);

    /* 全白 = 没线 → 停 */
    if (v[0] && v[1] && v[2] && v[3] && v[4] && v[5] && v[6] && v[7]) {
        Set_Pwm(0, 0);
        return;
    }

    /* 有线 → 用 Incremental_Quantity 加权偏差做比例差速 */
    int bias = Incremental_Quantity();
    float steer = (float)bias * 0.05f;      /* 缩放转向量 */

    float left  = TRACK_SPEED + steer;
    float right = TRACK_SPEED - steer;

    Set_Pwm((int)(left  * 100.0f),
            (int)(right * 100.0f));
}

/* ========== 定时中断中的控制函数（10ms） ========== */
static void Control(void)
{
    /* Task5 单独处理，不走状态机 */
    if (current_task == 5) {
        Control_Task5();
        return;
    }

    Follow_Route();
    update_encoder();
    Gyro_Update(0.01f);

    if (flag == 0 || !flag_en) {
        Set_Pwm(0, 0);
        PID_Reset(&speed_pid[0]);
        PID_Reset(&speed_pid[1]);
        return;
    }

    float target_speed[2];

    if (flag == 1 || flag == 3) {
        /* ---- 白区直线：里程PID + 角度PID（Jamie 方式） ---- */

        /* 1. 里程 PID → 基础速度 */
        mileage_pid.target = DIST_AB_PULSES;
        float base_speed = PID_Calculate(&mileage_pid, (float)encoder_left_cnt);

        /* 2. 角度 PID → 修正量 */
        float yaw_err = calc_angle_error(target_yaw, Gyro_GetYaw());
        straight_pid.target = 0.0f;
        float correction = PID_Calculate(&straight_pid, -yaw_err);

        /* 3. 左右轮目标速度 */
        target_speed[0] = base_speed + correction;
        target_speed[1] = base_speed - correction;

    } else if (flag == 2 || flag == 4) {
        /* ---- 半圆弧：循迹 ---- */
        float bias = TRACK_KP * (float)Incremental_Quantity();
        target_speed[0] = TRACK_SPEED + bias;
        target_speed[1] = TRACK_SPEED - bias;
    }

    /* 速度 PID → PWM 输出（已修复 double PID） */
    int pwm_left = 0, pwm_right = 0;
    for (int i = 0; i < 2; i++) {
        speed_pid[i].target = target_speed[i];
        float pwm = PID_Calculate(&speed_pid[i], actual_speed[i]);
        /* 限幅 */
        if (pwm >  PWM_LIMIT) pwm =  PWM_LIMIT;
        if (pwm < -PWM_LIMIT) pwm = -PWM_LIMIT;
        /* 死区补偿 */
        if (pwm > 0 && pwm < 200) pwm = 200;
        if (pwm < 0 && pwm > -200) pwm = -200;
        if (i == 0) pwm_left  = (int)pwm;
        else        pwm_right = (int)pwm;
    }
    Set_Pwm(pwm_left, pwm_right);
}

/* ========== 外部接口 ========== */

void Task_Init(uint8_t task)
{
    finished = 0;
    flag_en = 0;
    current_task = task;
    encoder_left_cnt = 0;
    encoder_right_cnt = 0;

    /* Task5 是纯循迹测试，不需要 PID 和陀螺仪 */
    if (task == 5) {
        Buzzer_ShortBeep();
        delay_ms(200);
        flag_en = 1;
        return;
    }

    /* 初始化 PID */
    PID_Init(&speed_pid[0],  SPEED_KP, SPEED_KI, SPEED_KD,  PWM_LIMIT, PWM_LIMIT);
    PID_Init(&speed_pid[1],  SPEED_KP, SPEED_KI, SPEED_KD,  PWM_LIMIT, PWM_LIMIT);
    PID_Init(&mileage_pid,   MILEAGE_KP, MILEAGE_KI, 0.0f,   5.0f, 5.0f);
    PID_Init(&straight_pid,  STRAIGHT_KP, 0.0f, STRAIGHT_KD, 10.0f, 10.0f);
    PID_Init(&track_pid,     TRACK_KP, 0.0f, 0.0f,           30.0f, 30.0f);

    /* 初始化陀螺仪 */
    Gyro_Init();
    target_yaw = Gyro_GetYaw();

    if (task == 1) {
        flag = 1;
    } else if (task == 2) {
        flag = 1;
    } else if (task == 3) {
        flag = 2;
    } else if (task == 4) {
        flag = 2;
    }

    Buzzer_ShortBeep();
    delay_ms(200);
    flag_en = 1;
}

void Task_Run(void)
{
    if (finished) return;
}

uint8_t Task_IsFinished(void)
{
    return finished;
}

/* ========== 定时中断 ========== */
void MOTOR_PID_INST_IRQHandler(void)
{
    switch (DL_Timer_getPendingInterrupt(MOTOR_PID_INST))
    {
    case DL_TIMER_IIDX_LOAD:
        Control();
        break;
    default:
        break;
    }
}
