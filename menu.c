#include "menu.h"
#include "oled.h"
#include "delay.h"

volatile uint8_t key_up_pressed   = 0;
volatile uint8_t key_down_pressed = 0;

/* 菜单显示文本 */
static const char *task_names[] = {
    "",
    "1: A->B      ",
    "2: A->B->C->D",
    "3: A->C->B->D",
    "4: x4 Laps   ",
};

void Menu_Init(void)
{
    key_up_pressed   = 0;
    key_down_pressed = 0;
}

/* 绘制菜单界面 */
static void Menu_Draw(uint8_t selected)
{
    OLED_Clear();
    OLED_ShowString(0, 0, (uint8_t *)"H-Task Select", 16);

    for (uint8_t i = 1; i <= 4; i++) {
        uint8_t y = 2 + (i - 1) * 2;  /* 行号 2,4,6,8（8像素行高） */
        if (i == selected) {
            OLED_ShowChar(0, y, '>', 16);
        } else {
            OLED_ShowChar(0, y, ' ', 16);
        }
        OLED_ShowString(12, y, (uint8_t *)task_names[i], 12);
    }
    OLED_Refresh();
}

/* 等待按键（带消抖），返回 1=UP, 2=DOWN */
static uint8_t Wait_Key(void)
{
    while (1) {
        if (key_up_pressed) {
            key_up_pressed = 0;
            delay_ms(50);  /* 消抖 */
            return 1;
        }
        if (key_down_pressed) {
            key_down_pressed = 0;
            delay_ms(50);
            return 2;
        }
    }
}

uint8_t Menu_SelectTask(void)
{
    uint8_t selected = 1;

    Menu_Draw(selected);

    while (1) {
        uint8_t key = Wait_Key();

        if (key == 1) {
            /* UP：上翻 */
            if (selected > 1)
                selected--;
            else
                selected = 4;
            Menu_Draw(selected);
        } else if (key == 2) {
            /* DOWN：确认 */
            return selected;
        }
    }
}

/* ========== 按键中断处理（GPIOB GROUP1） ========== */
/* 注意：encoder.c 中已定义 GROUP1_IRQHandler，需要合并 */
/* 这里通过标志位通知主循环，不在中断中做 OLED 操作 */
