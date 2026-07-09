/*
 * H题自动行驶小车 - 8路循迹版
 *
 * 控制方式：定时中断里做所有控制（循迹/直线 + 速度PID）
 *
 * =============================================
 *  改这里选任务：1 / 2 / 3 / 4
 * =============================================
 *  TASK_NUM 在 main.c 里
 *
 * =============================================
 *  可调参数速查
 * =============================================
 *
 *  【循迹速度】 → 第 40 行
 *    Speed_Middle = 30  ← 基础速度（PWM占空比 0~3800）
 *
 *  【循迹力度】 → 第 43 行
 *    KP_LINE = 10       ← 循迹比例系数，太大抖，太小偏
 *
 *  【速度PID】 → 第 46~48 行
 *    Kp1=40, Ki1=0.13, Kd1=0.1  ← 速度环PID
 *
 *  【直线段距离】 → 第 54~55 行
 *    DIST_AB_PULSES = 3000  ← A到B编码器脉冲数
 *    DIST_CD_PULSES = 3000  ← C到D编码器脉冲数
 *
 *  【传感器阈值】 → 第 58~62 行
 *    Black_CNT = 30   ← 连续检测到黑线N次才判定到达
 *    White_CNT = 1000 ← 连续检测到白区N次才判定离开
 */

#include "task.h"
#include "motor.h"
#include "ir_sensor.h"
#include "buzzer.h"
#include "oled.h"
#include "delay.h"
#include "encoder.h"

/* ---------- 速度参数 ---------- */
#define Speed_Middle    10.0f       /* 基础速度（PWM占空比） */
#define Limit           2000        /* PWM限幅 */
#define KP_LINE         8.0f       /* 循迹比例系数 */

/* ---------- 速度PID（增量式） ---------- */
#define Kp1             20.0f
#define Ki1             0.1f
#define Kd1             0.05f

/* ---------- 直线段距离（编码器脉冲数，需要实测） ---------- */
#define DIST_AB_PULSES  2400        /* A→B 脉冲数 */
#define DIST_CD_PULSES  2400        /* C→D 脉冲数 */

/* ---------- 分段判定阈值 ---------- */
#define Black_CNT       30          /* 连续检测到黑线次数 */
#define White_CNT       1000        /* 连续检测到白区次数 */

/* ---------- flag 定义 ----------
 * 0 = 停止
 * 1 = AB段（白区直线，编码器计距离）
 * 2 = BC段（半圆弧，循迹）
 * 3 = CD段（白区直线，编码器计距离）
 * 4 = DA段（半圆弧，循迹）
 * ---------------------------- */

static volatile int flag = 0;           /* 当前段 */
static volatile int flag_en = 0;        /* 使能标志 */
static volatile uint8_t finished = 0;

/* 编码器速度（10ms 内脉冲数） */
static volatile int32_t EncoderA_VEL = 0;  /* 左轮速度 */
static volatile int32_t EncoderB_VEL = 0;  /* 右轮速度 */

/* 增量式PID 状态 */
static float pid_left_pwm = 0;
static float pid_right_pwm = 0;

/* ---------- 增量式PID ---------- */
static float PID_Inc(float Encoder, float Target, float *last_bias, float *last2_bias)
{
    float Bias = Target - Encoder;
    float Pwm = Kp1 * (Bias - *last_bias) + Ki1 * Bias + Kd1 * (Bias - 2 * (*last_bias) + (*last2_bias));
    *last2_bias = *last_bias;
    *last_bias = Bias;
    return Pwm;
}

/* ---------- 限幅 ---------- */
static float PWM_Limit(float IN, float max, float min)
{
    if (IN > max) return max;
    if (IN < min) return min;
    return IN;
}

/* ---------- 设置电机PWM（直接操作方向引脚+PWM） ---------- */
static void Set_Pwm(int Left, int Right)
{
    if (Left > 0) {
        DL_GPIO_setPins(MOTOR_AIN1_PORT, MOTOR_AIN1_PIN);
        DL_GPIO_clearPins(MOTOR_AIN2_PORT, MOTOR_AIN2_PIN);
    } else {
        DL_GPIO_clearPins(MOTOR_AIN1_PORT, MOTOR_AIN1_PIN);
        DL_GPIO_setPins(MOTOR_AIN2_PORT, MOTOR_AIN2_PIN);
    }
    DL_TimerG_setCaptureCompareValue(PWMAB_INST, (uint32_t)(Left > 0 ? Left : -Left),
                                     DL_TIMER_CC_0_INDEX);

    if (Right > 0) {
        DL_GPIO_setPins(MOTOR_BIN1_PORT, MOTOR_BIN1_PIN);
        DL_GPIO_clearPins(MOTOR_BIN2_PORT, MOTOR_BIN2_PIN);
    } else {
        DL_GPIO_clearPins(MOTOR_BIN1_PORT, MOTOR_BIN1_PIN);
        DL_GPIO_setPins(MOTOR_BIN2_PORT, MOTOR_BIN2_PIN);
    }
    DL_TimerG_setCaptureCompareValue(PWMAB_INST, (uint32_t)(Right > 0 ? Right : -Right),
                                     DL_TIMER_CC_1_INDEX);
}

/* ---------- 8路循迹加权求和（参考 CAR/Sensor.c） ----------
 * S1=最右(+12), S2=+9, S3=+7, S4=+3,
 * S5=-3, S6=-7, S7=-9, S8=最左(-12)
 * 返回值：正=线偏右，负=线偏左
 * ----------------------------------------------- */
static int Incremental_Quantity(void)
{
    uint8_t v[IR_NUM];
    IR_Read(v);

    int value = 0;
    if (v[0]) value += 12;   /* S1 最右 */
    if (v[1]) value += 9;    /* S2 */
    if (v[2]) value += 7;    /* S3 */
    if (v[3]) value += 3;    /* S4 */
    if (v[4]) value -= 3;    /* S5 */
    if (v[5]) value -= 7;    /* S6 */
    if (v[6]) value -= 9;    /* S7 */
    if (v[7]) value -= 12;   /* S8 最左 */
    return value;
}

/* ---------- 读取编码器速度（10ms 调用一次） ---------- */
static void Read_Encoder(void)
{
    static int32_t last_left = 0, last_right = 0;

    int32_t cur_left = encoder_left_cnt;
    int32_t cur_right = encoder_right_cnt;

    EncoderA_VEL = cur_left - last_left;
    EncoderB_VEL = cur_right - last_right;

    last_left = cur_left;
    last_right = cur_right;
}

/* ---------- 分段判断（参考 CAR/Sensor.c Follow_Route） ---------- */
static void Follow_Route(void)
{
    static int cnt = 0;
    uint8_t v[IR_NUM];

    /* 任务1：A→B 直线 */
    if (flag == 1) {
        /* 编码器距离到了 */
        if (encoder_left_cnt >= DIST_AB_PULSES) {
            flag = 0;  /* 停止 */
            finished = 1;
        }
    }
    /* 任务2/3/4 的 BC 段（半圆弧循迹） */
    else if (flag == 2) {
        IR_Read(v);
        /* 全部白 → 离开黑线 → 到达下一个点 */
        if (!(v[0] || v[1] || v[2] || v[3] || v[4] || v[5] || v[6] || v[7])) {
            cnt++;
            if (cnt > White_CNT) {
                flag = 3;  /* 进入 CD 段 */
                cnt = 0;
            }
        } else {
            cnt = 0;
        }
    }
    /* CD 段（白区直线） */
    else if (flag == 3) {
        if (encoder_left_cnt >= DIST_AB_PULSES + DIST_CD_PULSES) {
            flag = 4;  /* 进入 DA 段 */
        }
    }
    /* DA 段（半圆弧循迹） */
    else if (flag == 4) {
        IR_Read(v);
        if (!(v[0] || v[1] || v[2] || v[3] || v[4] || v[5] || v[6] || v[7])) {
            cnt++;
            if (cnt > White_CNT) {
                flag = 0;  /* 一圈完成 */
                finished = 1;
                cnt = 0;
            }
        } else {
            cnt = 0;
        }
    }
}

/* ---------- 定时中断中的控制函数（参考 CAR/control.c Control） ---------- */
static void Control(void)
{
    float TargetA, TargetB;
    float bias = 0;

    /* 分段判断 */
    Follow_Route();

    if (flag == 0 || !flag_en) {
        Set_Pwm(0, 0);
        pid_left_pwm = 0;
        pid_right_pwm = 0;
        return;
    }

    /* 根据当前段选择控制方式 */
    if (flag == 1 || flag == 3) {
        /* 白区直线：直走，不加循迹修正 */
        bias = -3;
    } else if (flag == 2 || flag == 4) {
        /* 半圆弧：循迹 */
        bias = KP_LINE * (float)Incremental_Quantity();
    }

    /* 计算目标速度 */
    TargetA = Speed_Middle - bias;   /* 左轮 */
    TargetB = Speed_Middle + bias;   /* 右轮 */

    /* 读取编码器 */
    Read_Encoder();

    /* 增量式PID */
    static float last_bias_a = 0, last2_bias_a = 0;
    static float last_bias_b = 0, last2_bias_b = 0;

    pid_left_pwm  += PID_Inc((float)EncoderA_VEL, TargetA, &last_bias_a, &last2_bias_a);
    pid_right_pwm += PID_Inc((float)EncoderB_VEL, TargetB, &last_bias_b, &last2_bias_b);

    /* 限幅 */
    pid_left_pwm  = PWM_Limit(pid_left_pwm, Limit, -Limit);
    pid_right_pwm = PWM_Limit(pid_right_pwm, Limit, -Limit);

    /* 输出 */
    Set_Pwm((int)pid_left_pwm, (int)pid_right_pwm);
}

/* ========== 外部接口 ========== */

void Task_Init(uint8_t task)
{
    finished = 0;
    flag_en = 0;
    pid_left_pwm = 0;
    pid_right_pwm = 0;
    encoder_left_cnt = 0;
    encoder_right_cnt = 0;

    if (task == 1) {
        flag = 1;   /* A→B 直线 */
    } else if (task == 2) {
        flag = 1;   /* A→B→C→D→A，从 AB 直线开始 */
    } else if (task == 3) {
        flag = 2;   /* A→C→B→D→A，从半圆弧开始 */
    } else if (task == 4) {
        flag = 2;   /* 4圈，从半圆弧开始 */
    }

    Buzzer_ShortBeep();
    delay_ms(200);
    flag_en = 1;  /* 使能控制 */
}

void Task_Run(void)
{
    /* 控制在定时中断里做，这里只检查是否完成 */
    if (finished) return;
}

uint8_t Task_IsFinished(void)
{
    return finished;
}

/* ========== 定时中断处理（替代 motor.c 的中断） ========== */
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
