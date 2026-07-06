#include "ir_sensor.h"

/* 5路循迹传感器 GPIO 引脚（可能在不同端口） */
static const struct {
    GPIO_Regs *port;
    uint32_t pin;
} ir_sensors[IR_NUM] = {
    { HUIDU_L2_PORT, HUIDU_L2_PIN },   /* PA17 */
    { HUIDU_L1_PORT, HUIDU_L1_PIN },   /* PB8  */
    { HUIDU_M_PORT,  HUIDU_M_PIN  },   /* PB9  */
    { HUIDU_R1_PORT, HUIDU_R1_PIN },   /* PA24 */
    { HUIDU_R2_PORT, HUIDU_R2_PIN },   /* PA2  */
};

void IR_Init(void) { /* SysConfig 已配置 GPIO 输入 */ }

void IR_Read(uint8_t *values)
{
    for (uint8_t i = 0; i < IR_NUM; i++) {
        values[i] = DL_GPIO_readPins(ir_sensors[i].port, ir_sensors[i].pin) ? 0 : 1;
    }
}

/* 加权平均计算偏差，返回 -400 ~ +400 */
int16_t IR_GetError(uint8_t *values)
{
    static const int16_t weight[IR_NUM] = {-400, -200, 0, 200, 400};
    int32_t sum = 0;
    uint8_t cnt = 0;
    for (uint8_t i = 0; i < IR_NUM; i++) {
        if (values[i]) { sum += weight[i]; cnt++; }
    }
    return cnt ? (int16_t)(sum / cnt) : 0;
}
