#include "encoder.h"
#include "motor.h"   /* ENCODER_*_PORT / PIN 宏 */

/* ---------- 脉冲计数（中断写，主循环/定时中断读） ---------- */
volatile int32_t encoder_left_cnt  = 0;
volatile int32_t encoder_right_cnt = 0;

/* ---------- 初始化：使能编码器 GPIO 中断 ---------- */
void Encoder_Init(void)
{
    encoder_left_cnt  = 0;
    encoder_right_cnt = 0;

    /* SysConfig 已配置 interruptEn + polarity=RISE，这里只使能 NVIC */
    NVIC_EnableIRQ(GPIOA_INT_IRQn);   /* PA21 (左编码器 A) */
    NVIC_EnableIRQ(GPIOB_INT_IRQn);   /* PB19 (右编码器 A) */
}

/* ========== GPIO 中断处理（GPIOA 和 GPIOB 都在 GROUP1） ========== */
void GROUP1_IRQHandler(void)
{
    /* 左编码器：A=PA21 触发，读 B=PA22 判断方向 */
    uint32_t pendingA = DL_GPIO_getEnabledInterruptStatus(GPIOA, 0xFFFFFFFF);
    if (pendingA & ENCODER_LA_PIN) {
        if (DL_GPIO_readPins(ENCODER_LB_PORT, ENCODER_LB_PIN))
            encoder_left_cnt++;
        else
            encoder_left_cnt--;
        DL_GPIO_clearInterruptStatus(GPIOA, ENCODER_LA_PIN);
    }

    /* 右编码器：A=PB19 触发，读 B=PB20 判断方向 */
    uint32_t pendingB = DL_GPIO_getEnabledInterruptStatus(GPIOB, 0xFFFFFFFF);
    if (pendingB & ENCODER_RA_PIN) {
        if (DL_GPIO_readPins(ENCODER_RB_PORT, ENCODER_RB_PIN))
            encoder_right_cnt++;
        else
            encoder_right_cnt--;
        DL_GPIO_clearInterruptStatus(GPIOB, ENCODER_RA_PIN);
    }
}
