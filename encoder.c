#include "encoder.h"
#include "motor.h"   /* ENCODER_*_PORT / PIN 宏 */

/* ---------- 脉冲计数（中断写，主循环/定时中断读） ---------- */
volatile int32_t encoder_left_cnt  = 0;
volatile int32_t encoder_right_cnt = 0;

/* 上一次读数（供 Update 内部使用） */
static int32_t last_left  = 0;
static int32_t last_right = 0;

/* ---------- 初始化：使能编码器 GPIO 中断 ---------- */
void Encoder_Init(void)
{
    encoder_left_cnt  = 0;
    encoder_right_cnt = 0;
    last_left  = 0;
    last_right = 0;

    /* SysConfig 已配置 interruptEn + polarity=RISE，这里只使能 NVIC */
    NVIC_EnableIRQ(GPIOA_INT_IRQn);   /* PA21 (左编码器 A) */
    NVIC_EnableIRQ(GPIOB_INT_IRQn);   /* PB19 (右编码器 A) */
}

/* ---------- 定时中断中调用：返回本次脉冲增量并清零 ---------- */
void Encoder_Update(void)
{
    last_left  = encoder_left_cnt;
    last_right = encoder_right_cnt;
    /* 注意：不在这里清零，让 PID 直接读 cnt 差值 */
}

/* ========== GPIO 中断处理（GPIOA 和 GPIOB 都在 GROUP1） ========== */
/* 左编码器 A=PA21, 右编码器 A=PB19 */
void GROUP1_IRQHandler(void)
{
    /* 左编码器 A (PA21) */
    uint32_t pendingA = DL_GPIO_getEnabledInterruptStatus(GPIOA, 0xFFFFFFFF);
    if (pendingA & ENCODER_LA_PIN) {
        if (DL_GPIO_readPins(ENCODER_LB_PORT, ENCODER_LB_PIN))
            encoder_left_cnt++;
        else
            encoder_left_cnt--;
        DL_GPIO_clearInterruptStatus(GPIOA, ENCODER_LA_PIN);
    }

    /* 右编码器 A (PB19) */
    uint32_t pendingB = DL_GPIO_getEnabledInterruptStatus(GPIOB, 0xFFFFFFFF);
    if (pendingB & ENCODER_RA_PIN) {
        if (DL_GPIO_readPins(ENCODER_RB_PORT, ENCODER_RB_PIN))
            encoder_right_cnt++;
        else
            encoder_right_cnt--;
        DL_GPIO_clearInterruptStatus(GPIOB, ENCODER_RA_PIN);
    }
}
