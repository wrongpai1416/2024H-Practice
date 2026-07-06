/*
 * H题任务状态机
 *
 * 场地布局：
 *        A ←——— 直线 ———→ B
 *        |                   |
 *     左半圆              右半圆
 *     (R=40cm)            (R=40cm)
 *        |                   |
 *        C ←——— 直线 ———→ D
 *
 * 任务1：A→B 直线停车
 * 任务2：A→B→C(右半圆)→D→A(左半圆)
 * 任务3：A→C(左半圆)→B→D(右半圆)→A
 * 任务4：任务3路径 ×4圈
 */

#include "task.h"
#include "motor.h"
#include "ir_sensor.h"
#include "gyro.h"
#include "buzzer.h"
#include "oled.h"
#include "delay.h"

/* ---------- 参数可调 ---------- */
#define BASE_SPEED          25.0f   /* 直线基础速度（脉冲/10ms） */
#define LOST_SPEED          10.0f   /* 丢线低速 */
#define TURN_SPEED          18.0f   /* 半圆弧基础速度 */
#define TURN_TARGET_ANGLE   175.0f  /* 半圆弧目标转角（度，略小于180做补偿） */
#define TURN_ANGLE_TOL      8.0f   /* 转角容差（度） */
#define TURN_KP             0.8f   /* 转向 PID - Kp */
#define TURN_KI             0.0f   /* 转向 PID - Ki */
#define TURN_KD             0.3f   /* 转向 PID - Kd */
#define CURVE_ENTER_THRESH  180.0f /* 进入半圆弧的循迹误差阈值 */
#define CURVE_ENTER_COUNT   3      /* 连续检测到弯道的次数 */
#define STRAIGHT_TICKS      250    /* 直线段行驶 tick 数（×10ms = 2.5s） */
#define POINT_DELAY_MS      300    /* 到达点位后的声光持续时间 */

/* ---------- 内部状态 ---------- */
static TaskState  state        = STATE_IDLE;
static uint8_t    task_num     = 0;
static uint8_t    finished     = 0;

/* 任务序列：路径点数和每步类型 */
/* step_type: 0=直线, 1=右半圆, -1=左半圆 */
#define MAX_STEPS 8
static int8_t  step_types[MAX_STEPS];  /* 每步类型 */
static uint8_t step_count = 0;         /* 总步数 */
static uint8_t step_idx   = 0;         /* 当前步 */

/* 半圆弧转向 PID 状态 */
static float turn_integral   = 0.0f;
static float turn_prev_error = 0.0f;

/* 半圆弧进入检测 */
static uint8_t curve_enter_cnt = 0;

/* 直线段计时 */
static uint16_t straight_ticks = 0;

/* 圈数计数（任务4用） */
static uint8_t lap_count = 0;
static uint8_t total_laps = 1;

/* ---------- 内部函数 ---------- */

/* 构建任务路径 */
static void BuildPath(uint8_t task)
{
    step_idx = 0;
    lap_count = 0;

    if (task == 1) {
        /* A→B：直线 */
        step_types[0] = 0;
        step_count = 1;
        total_laps = 1;
    }
    else if (task == 2) {
        /* A→B(直)→C(右半圆)→D(直)→A(左半圆) */
        step_types[0] = 0;   /* A→B 直线 */
        step_types[1] = 1;   /* B→C 右半圆 */
        step_types[2] = 0;   /* C→D 直线 */
        step_types[3] = -1;  /* D→A 左半圆 */
        step_count = 4;
        total_laps = 1;
    }
    else if (task == 3 || task == 4) {
        /* A→C(左半圆)→B(直)→D(右半圆)→A(直) */
        step_types[0] = -1;  /* A→C 左半圆 */
        step_types[1] = 0;   /* C→B 直线 */
        step_types[2] = 1;   /* B→D 右半圆 */
        step_types[3] = 0;   /* D→A 直线 */
        step_count = 4;
        total_laps = (task == 4) ? 4 : 1;
    }
}

/* 声光提示 */
static void SignalArrival(void)
{
    Buzzer_ShortBeep();
}

/* 直线循迹控制（返回循迹误差绝对值） */
static float LineFollowStep(void)
{
    uint8_t vals[IR_NUM];
    IR_Read(vals);
    int16_t error = IR_GetError(vals);

    /* 全白 → 丢线，低速直走 */
    if (vals[0] == 0 && vals[1] == 0 && vals[2] == 0 &&
        vals[3] == 0 && vals[4] == 0) {
        Motor_SetTarget(LOST_SPEED, LOST_SPEED);
        return 0.0f;
    }

    /* 分段修正循迹 */
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

    return abs_err;
}

/* 半圆弧转向控制 */
static void SemiTurnStep(TurnDir dir)
{
    float yaw = Gyro_GetYaw();
    float angle_error;

    if (dir == TURN_RIGHT) {
        /* 右半圆：yaw 从 0 增加到 +180 */
        angle_error = TURN_TARGET_ANGLE - yaw;
    } else {
        /* 左半圆：yaw 从 0 减小到 -180 */
        angle_error = (-TURN_TARGET_ANGLE) - yaw;
    }

    /* PID 计算 */
    turn_integral += angle_error;
    if (turn_integral >  500.0f) turn_integral =  500.0f;
    if (turn_integral < -500.0f) turn_integral = -500.0f;

    float derivative = angle_error - turn_prev_error;
    turn_prev_error = angle_error;

    float pid_out = TURN_KP * angle_error
                  + TURN_KI * turn_integral
                  + TURN_KD * derivative;

    /* 差速控制 */
    float left_speed  = TURN_SPEED + pid_out;
    float right_speed = TURN_SPEED - pid_out;

    Motor_SetTarget(left_speed, right_speed);
}

/* 检测半圆弧是否完成 */
static uint8_t IsSemiTurnDone(TurnDir dir)
{
    float yaw = Gyro_GetYaw();

    if (dir == TURN_RIGHT) {
        return (yaw >= (TURN_TARGET_ANGLE - TURN_ANGLE_TOL));
    } else {
        return (yaw <= (-TURN_TARGET_ANGLE + TURN_ANGLE_TOL));
    }
}

/* 进入下一步 */
static void EnterNextStep(void)
{
    step_idx++;

    /* 一圈的步数走完 → 检查是否需要下一圈 */
    if (step_idx >= step_count) {
        lap_count++;
        if (lap_count >= total_laps) {
            state = STATE_STOP;
            return;
        }
        step_idx = 0;  /* 下一圈从第0步开始 */
    }

    /* 根据下一步类型进入对应状态 */
    int8_t next_type = step_types[step_idx];

    if (next_type == 0) {
        /* 下一步是直线：先走半圆弧出口的直线段 */
        /* 但这里 step_idx 已经指向直线步了，所以直接进入直线循迹 */
        state = STATE_LINE_FOLLOW;
        straight_ticks = 0;
    } else {
        /* 下一步是半圆弧：进入弯道检测模式 */
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

    /* 初始化第一步 */
    int8_t first_type = step_types[0];
    if (first_type == 0) {
        state = STATE_LINE_FOLLOW;
        straight_ticks = 0;
    } else {
        state = STATE_SEMI_TURN;
        Gyro_Reset();
        turn_integral = 0.0f;
        turn_prev_error = 0.0f;
    }

    curve_enter_cnt = 0;

    /* 开始前声光提示 */
    SignalArrival();
    delay_ms(200);
}

void Task_Run(void)
{
    if (finished) return;

    switch (state) {

    case STATE_LINE_FOLLOW: {
        /*
         * 当前步是直线：循迹行驶，计时到达后进入 ARRIVED
         * 当前步是半圆弧（从 ARRIVED 回来后）：循迹行驶同时检测弯道入口
         */
        float abs_err = LineFollowStep();

        int8_t cur_type = step_types[step_idx];

        if (cur_type == 0) {
            /* 当前步是直线 → 用计时判断到达 */
            straight_ticks++;
            if (straight_ticks >= STRAIGHT_TICKS) {
                straight_ticks = 0;
                state = STATE_ARRIVED;
            }
        } else {
            /* 当前步是半圆弧 → 检测弯道入口（误差突然增大） */
            if (abs_err > CURVE_ENTER_THRESH) {
                curve_enter_cnt++;
                if (curve_enter_cnt >= CURVE_ENTER_COUNT) {
                    /* 确认进入半圆弧 */
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
        /* 声光提示 */
        SignalArrival();

        /* 短暂停顿让车稳定 */
        Motor_SetTarget(BASE_SPEED * 0.5f, BASE_SPEED * 0.5f);
        delay_ms(POINT_DELAY_MS);

        /* 前进一步到下一步 */
        EnterNextStep();
        break;
    }

    case STATE_STOP: {
        Motor_Stop();
        /* 最终声光提示：闪烁3次 */
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
