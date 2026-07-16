#ifndef __OLED_H
#define __OLED_H

#include "ti_msp_dl_config.h"

#define OLED_CMD  0
#define OLED_DATA 1

void OLED_Init(void);
void OLED_ColorTurn(uint8_t i);
void OLED_DisplayTurn(uint8_t i);
void OLED_WR_Byte(uint8_t dat,uint8_t mode);
void OLED_DisPlay_On(void);
void OLED_DisPlay_Off(void);
void OLED_Refresh(void);
void OLED_Clear(void);
void OLED_DrawPoint(uint8_t x,uint8_t y);
void OLED_ClearPoint(uint8_t x,uint8_t y);
void OLED_DrawLine(uint8_t x1,uint8_t y1,uint8_t x2,uint8_t y2);
void OLED_DrawCircle(uint8_t x,uint8_t y,uint8_t r);
void OLED_ShowChar(uint8_t x,uint8_t y,uint8_t chr,uint8_t size1);
void OLED_ShowString(uint8_t x,uint8_t y,uint8_t *chr,uint8_t size1);
void OLED_ShowNum(uint8_t x,uint8_t y,uint32_t num,uint8_t len,uint8_t size1);
void OLED_ShowChinese(uint8_t x,uint8_t y,uint8_t num,uint8_t size1);
void OLED_ShowPicture(uint8_t x0,uint8_t y0,uint8_t x1,uint8_t y1,uint8_t BMP[]);

#endif
