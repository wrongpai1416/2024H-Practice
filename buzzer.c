#include "buzzer.h"
#include "delay.h"

void Buzzer_Init(void)
{
    /* GPIO 已由 SysConfig 配置，确保 LED 灭 */
    DL_GPIO_clearPins(LED_PORT, LED_D0_PIN);
}

void Buzzer_Beep(uint16_t ms)
{
    DL_GPIO_setPins(LED_PORT, LED_D0_PIN);
    delay_ms(ms);
    DL_GPIO_clearPins(LED_PORT, LED_D0_PIN);
}

void Buzzer_ShortBeep(void)
{
    Buzzer_Beep(200);
}
