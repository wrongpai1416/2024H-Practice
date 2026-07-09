#include "ir_sensor.h"

/* 8路循迹传感器 GPIO 引脚 */
static const struct {
    GPIO_Regs *port;
    uint32_t pin;
} ir_sensors[IR_NUM] = {
    { HUIDU_S1_PORT, HUIDU_S1_PIN },   /* PA18 - 最右 */
    { HUIDU_S2_PORT, HUIDU_S2_PIN },   /* PB8  */
    { HUIDU_S3_PORT, HUIDU_S3_PIN },   /* PB9  */
    { HUIDU_S4_PORT, HUIDU_S4_PIN },   /* PA23 */
    { HUIDU_S5_PORT, HUIDU_S5_PIN },   /* PA24 */
    { HUIDU_S6_PORT, HUIDU_S6_PIN },   /* PA25 */
    { HUIDU_S7_PORT, HUIDU_S7_PIN },   /* PA26 */
    { HUIDU_S8_PORT, HUIDU_S8_PIN },   /* PA27 - 最左 */
};

void IR_Init(void) { /* SysConfig 已配置 GPIO 输入 */ }

void IR_Read(uint8_t *values)
{
    for (uint8_t i = 0; i < IR_NUM; i++) {
        values[i] = DL_GPIO_readPins(ir_sensors[i].port, ir_sensors[i].pin) ? 1 : 0;
    }
}

/* 加权平均计算偏差，返回 -700 ~ +700 */
int16_t IR_GetError(uint8_t *values)
{
    static const int16_t weight[IR_NUM] = {-700, -500, -300, -100, 100, 300, 500, 700};
    int32_t sum = 0;
    uint8_t cnt = 0;
    for (uint8_t i = 0; i < IR_NUM; i++) {
        if (values[i]) { sum += weight[i]; cnt++; }
    }
    return cnt ? (int16_t)(sum / cnt) : 0;
}
