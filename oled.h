#ifndef __OLED_H
#define __OLED_H 

#include "ti_msp_dl_config.h"

#define OLED_CMD  0
#define OLED_DATA 1

void OLED_Init(void); //初始化OLED
void OLED_ColorTurn(uint8_t i); //是否反色显示
void OLED_DisplayTurn(uint8_t i); //是否旋转 180 度显示
void OLED_WR_Byte(uint8_t dat,uint8_t mode); //写入一个字节的数据或命令
void OLED_DisPlay_On(void); //打开显示
void OLED_DisPlay_Off(void); //关闭显示
void OLED_Refresh(void); //显存刷新至屏幕
void OLED_Clear(void); //清空显存并刷新到屏幕
void OLED_DrawPoint(uint8_t x,uint8_t y); //在显存中绘制一个像素
void OLED_ClearPoint(uint8_t x,uint8_t y); //清除显存中的一个像素
void OLED_DrawLine(uint8_t x1,uint8_t y1,uint8_t x2,uint8_t y2); //在显存中绘制直线
void OLED_DrawCircle(uint8_t x,uint8_t y,uint8_t r); //在显存中绘制圆
void OLED_ShowChar(uint8_t x,uint8_t y,uint8_t chr,uint8_t size1); //在显存中绘制单个字符
void OLED_ShowString(uint8_t x,uint8_t y,uint8_t *chr,uint8_t size1); //在显存中绘制字符串
void OLED_ShowNum(uint8_t x,uint8_t y,uint32_t num,uint8_t len,uint8_t size1); //在显存中绘制无符号整数
void OLED_ShowChinese(uint8_t x,uint8_t y,uint8_t num,uint8_t size1); //在显存中绘制汉字
void OLED_ShowPicture(uint8_t x0,uint8_t y0,uint8_t x1,uint8_t y1,uint8_t BMP[]); //在指定位置绘制图像

#endif
