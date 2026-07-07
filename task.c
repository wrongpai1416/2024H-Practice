/*
 * H题任务状态机
 *
 * 场地布局（只有半圆弧是黑线，直线段是白的）：
 *        A                   B
 *        |                   |
 *     左半圆(A-C)         右半圆(B-D)
 *        |                   |
 *        C                   D
 *
 * 任务1：A→B（白区直线，编码器+陀螺仪）
 * 任务2：A→B→C→D→A
 * 任务3：A→C→B→D→A
 * 任务4：任务3路径 ×4圈
 *
 * 直线段：陀螺仪保持方向 + 编码器计距离
 * 半圆弧段：循迹检测进入 → 陀螺仪转180° → 循迹出来
 */

#include "task.h"
#include "motor.h"
#include "ir_sensor.h"
#include "gyro.h"
#include "buzzer.h"
#include "oled.h"
#include "delay.h"
#include "encoder.h"

/* ---------- 参数可调 ---------- */
#define BASE_SPEED          20.0f   /* 直线基础速度 */
#define LOST_SPEED          10.0f   /* 丢线低速 */
#define TURN_SPEED          18.0f   /* 半圆弧基础速度 */
#define TURN_TARGET_ANGLE   175.0f  /* 半圆弧目标转角（度） */
#define TURN_ANGLE_TOL      8.0f   /* 转角容差 */
#define TURN_KP             0.8f   /* 转向 Kp */
#define TURN_KD             0.3f   /* 转向 Kd */
#define CURVE_ENTER_THRESH  180.0f /* 进入半圆弧的循迹误差阈值 */
#define CURVE_ENTER_COUNT   3      /* 连续检测到弯道次数 */
#define POINT_DELAY_MS      300    /* 到达点位声光持续时间 */

/* 直线段参数 */
#define STRAIGHT_KP         2.5f   /* 直线保持 Kp */
#define PULSES_PER_CM       100.0f /* 编码器脉冲/厘米（实测值） */
#define MAX_STRAIGHT_MS     15000  /* 直线段最大时间(ms) */

/* 直线保持参数：编码器累计差 + 陀螺仪航向 */
#define ENC_DIFF_KP         0.35f  /* 编码器累计差 P */
#define ENC_DIFF_KI         0.03f  /* 编码器累计差 I */
#define ENC_DIFF_KD         0.15f  /* 编码器瞬时差 D */
#define ENC_DIFF_MAX_I      3.0f   /* I 项限幅 */
#define GYRO_KP             1.6f   /* 陀螺仪航向 P */
#define GYRO_KD             0.4f   /* 陀螺仪航向 D */
#define STEERING_MAX        5.0f   /* 总修正量限幅 */

/* 各段距离（厘米） */
#define DIST_AB_CM          15.0f
#define DIST_CD_CM          15.0f

/* ---------- 内部状态 ---------- */
static TaskState  state        = STATE_IDLE;
static uint8_t    task_num     = 0;
static uint8_t    finished     = 0;

/* 路径定义: 0=直线, 1=右半圆, -1=左半圆 */
#define MAX_STEPS 8
static int8_t   step_types[MAX_STEPS];
static uint16_t step_dist_pulses[MAX_STEPS];
static uint8_t  step_count = 0;
static uint8_t  step_idx   = 0;

/* 半圆弧转向 PID */
static float turn_integral   = 0.0f;
static float turn_prev_error = 0.0f;
static uint8_t curve_enter_cnt = 0;

/* 直线段 */
static uint32_t straight_pulses = 0;
static int32_t  straight_start_left = 0;  /* 起始左编码器值 */
static float    straight_heading = 0.0f;
static uint32_t straight_ms = 0;
static float    enc_diff_integral = 0.0f;   /* 编码器累计差 I 项 */
static int32_t  prev_total_diff   = 0;      /* 上一帧累计差（用于 D 项） */
static float    gyro_prev_error   = 0.0f;   /* 陀螺仪航向上一帧误差 */

/* 圈数 */
static uint8_t lap_count = 0;
static uint8_t total_laps = 1;

/* ---------- 内部函数 ---------- */

static void BuildPath(uint8_t task)
{
    step_idx = 0;
    lap_count = 0;

    uint16_t dist_ab = (uint16_t)(DIST_AB_CM * PULSES_PER_CM);
    uint16_t dist_cd = (uint16_t)(DIST_CD_CM * PULSES_PER_CM);

    if (task == 1) {
        step_types[0] = 0;
        step_dist_pulses[0] = dist_ab;
        step_count = 1;
        total_laps = 1;
    }
    else if (task == 2) {
        step_types[0] = 0;   step_dist_pulses[0] = dist_ab;   /* A→B */
        step_types[1] = 1;                                      /* B→D 右半圆 */
        step_types[2] = 0;   step_dist_pulses[2] = dist_cd;   /* D→C */
        step_types[3] = -1;                                     /* C→A 左半圆 */
        step_count = 4;
        total_laps = 1;
    }
    else if (task == 3 || task == 4) {
        step_types[0] = -1;                                     /* A→C 左半圆 */
        step_types[1] = 0;   step_dist_pulses[1] = dist_cd;   /* C→B */
        step_types[2] = 1;                                      /* B→D 右半圆 */
        step_types[3] = 0;   step_dist_pulses[3] = dist_ab;   /* D→A */
        step_count = 4;
        total_laps = (task == 4) ? 4 : 1;
    }
}

static void SignalArrival(void)
{
    Buzzer_ShortBeep();
}

/* 白区直线行驶：编码器(PID) + 陀螺仪(PD) 双重修正 */
static int32_t prev_left = 0, prev_right = 0;

static void StraightStep(void)
{
    straight_ms += 10;

    /* 用单边编码器计距离（取绝对值，防转圈时不算距离） */
    int32_t cur_left = encoder_left_cnt;
    int32_t diff_left = cur_left - straight_start_left;
    if (diff_left < 0) diff_left = -diff_left;
    straight_pulses = (uint32_t)diff_left;

    /* ---- 1. 编码器累计位移差(PID) ---- */
    int32_t total_diff = encoder_left_cnt - encoder_right_cnt;

    enc_diff_integral += (float)total_diff;
    if (enc_diff_integral >  ENC_DIFF_MAX_I) enc_diff_integral =  ENC_DIFF_MAX_I;
    if (enc_diff_integral < -ENC_DIFF_MAX_I) enc_diff_integral = -ENC_DIFF_MAX_I;

    float d_term = (float)(total_diff - prev_total_diff);
    prev_total_diff = total_diff;

    float enc_correction = ENC_DIFF_KP * (float)total_diff
                         + ENC_DIFF_KI * enc_diff_integral
                         + ENC_DIFF_KD * d_term;

    /* ---- 2. 陀螺仪航向修正(PD) ---- */
    float heading_err = Gyro_GetYaw() - straight_heading;
    float heading_d = heading_err - gyro_prev_error;
    gyro_prev_error = heading_err;

    float gyro_correction = -(GYRO_KP * heading_err
                           + GYRO_KD * heading_d);

    /* ---- 3. 合并 ---- */
    float correction = enc_correction + gyro_correction;
    if (correction >  STEERING_MAX) correction =  STEERING_MAX;
    if (correction < -STEERING_MAX) correction = -STEERING_MAX;

    Motor_SetTarget(BASE_SPEED + correction,
                    BASE_SPEED - correction);
}

static uint8_t IsStraightDone(void)
{
    /* 编码器距离到了 */
    if (straight_pulses >= step_dist_pulses[step_idx])
        return 1;
    /* 超时保底 */
    if (straight_ms >= MAX_STRAIGHT_MS)
        return 1;
    return 0;
}

/* 循迹控制 */
static float LineFollowStep(void)
{
    uint8_t vals[IR_NUM];
    IR_Read(vals);
    int16_t error = IR_GetError(vals);

    if (vals[0] == 0 && vals[1] == 0 && vals[2] == 0 &&
        vals[3] == 0 && vals[4] == 0) {
        Motor_SetTarget(LOST_SPEED, LOST_SPEED);
        return 0.0f;
    }

    float abs_err = (float)(error > 0 ? error : -error);
    float correction;

    if (abs_err < 100.0f)
        correction = abs_err * 0.02f;
    else if (abs_err < 250.0f)
        correction = abs_err * 0.07f;
    else
        correction = abs_err * 0.15f;

    if (error > 0)
        Motor_SetTarget(BASE_SPEED + correction, BASE_SPEED - correction);
    else
        Motor_SetTarget(BASE_SPEED - correction, BASE_SPEED + correction);

    return abs_err;
}

/* 半圆弧转向 */
static void SemiTurnStep(TurnDir dir)
{
    float yaw = Gyro_GetYaw();
    float angle_error;

    if (dir == TURN_RIGHT)
        angle_error = TURN_TARGET_ANGLE - yaw;
    else
        angle_error = (-TURN_TARGET_ANGLE) - yaw;

    turn_integral += angle_error;
    if (turn_integral >  500.0f) turn_integral =  500.0f;
    if (turn_integral < -500.0f) turn_integral = -500.0f;

    float derivative = angle_error - turn_prev_error;
    turn_prev_error = angle_error;

    float pid_out = TURN_KP * angle_error
                  + 0.0f * turn_integral
                  + TURN_KD * derivative;

    Motor_SetTarget(TURN_SPEED + pid_out, TURN_SPEED - pid_out);
}

static uint8_t IsSemiTurnDone(TurnDir dir)
{
    float yaw = Gyro_GetYaw();
    if (dir == TURN_RIGHT)
        return (yaw >= (TURN_TARGET_ANGLE - TURN_ANGLE_TOL));
    else
        return (yaw <= (-TURN_TARGET_ANGLE + TURN_ANGLE_TOL));
}

static void EnterNextStep(void)
{
    step_idx++;

    if (step_idx >= step_count) {
        lap_count++;
        if (lap_count >= total_laps) {
            state = STATE_STOP;
            return;
        }
        step_idx = 0;
    }

    int8_t next_type = step_types[step_idx];

    if (next_type == 0) {
        state = STATE_LINE_FOLLOW;
        straight_start_left = encoder_left_cnt;
        straight_pulses = 0;
        straight_ms = 0;
        prev_left = encoder_left_cnt;
        prev_right = encoder_right_cnt;
        straight_heading = Gyro_GetYaw();
        enc_diff_integral = 0.0f;
        prev_total_diff = 0;
        gyro_prev_error = 0.0f;
    } else {
        state = STATE_SEMI_TURN;
        Gyro_Reset();
        turn_integral = 0.0f;
        turn_prev_error = 0.0f;
        curve_enter_cnt = 0;
    }
}

/* ---------- 外部接口 ---------- */

void Task_Init(uint8_t task)
{
    task_num  = task;
    finished  = 0;
    lap_count = 0;

    BuildPath(task);

    int8_t first_type = step_types[0];
    if (first_type == 0) {
        state = STATE_LINE_FOLLOW;
        straight_start_left = encoder_left_cnt;
        straight_pulses = 0;
        straight_ms = 0;
        prev_left = encoder_left_cnt;
        prev_right = encoder_right_cnt;
        straight_heading = Gyro_GetYaw() - 10.0f;  /* 左偏30°补偿右漂 */
        enc_diff_integral = 0.0f;
        prev_total_diff = 0;
        gyro_prev_error = 0.0f;
    } else {
        state = STATE_SEMI_TURN;
        Gyro_Reset();
        turn_integral = 0.0f;
        turn_prev_error = 0.0f;
    }

    curve_enter_cnt = 0;

    SignalArrival();
    delay_ms(200);
}

void Task_Run(void)
{
    if (finished) return;

    switch (state) {

    case STATE_LINE_FOLLOW: {
        int8_t cur_type = step_types[step_idx];

        if (cur_type == 0) {
            /* 白区直线 */
            StraightStep();
            if (IsStraightDone()) {
                straight_ms = 0;
                state = STATE_ARRIVED;
            }
        } else {
            /* 半圆弧前的循迹段 */
            float abs_err = LineFollowStep();
            if (abs_err > CURVE_ENTER_THRESH) {
                curve_enter_cnt++;
                if (curve_enter_cnt >= CURVE_ENTER_COUNT) {
                    state = STATE_SEMI_TURN;
                    Gyro_Reset();
                    turn_integral = 0.0f;
                    turn_prev_error = 0.0f;
                    curve_enter_cnt = 0;
                }
            } else {
                curve_enter_cnt = 0;
            }
        }
        break;
    }

    case STATE_SEMI_TURN: {
        TurnDir dir = (step_types[step_idx] > 0) ? TURN_RIGHT : TURN_LEFT;
        SemiTurnStep(dir);
        if (IsSemiTurnDone(dir)) {
            state = STATE_ARRIVED;
        }
        break;
    }

    case STATE_ARRIVED: {
        SignalArrival();
        Motor_SetTarget(BASE_SPEED * 0.5f, BASE_SPEED * 0.5f);
        delay_ms(POINT_DELAY_MS);
        EnterNextStep();
        break;
    }

    case STATE_STOP: {
        NVIC_DisableIRQ(MOTOR_PID_INST_INT_IRQN);
        Motor_Stop();
        for (int i = 0; i < 3; i++) {
            Buzzer_ShortBeep();
            delay_ms(100);
        }
        finished = 1;
        state = STATE_IDLE;
        break;
    }

    case STATE_IDLE:
    default:
        break;
    }
}

uint8_t Task_IsFinished(void)
{
    return finished;
}
