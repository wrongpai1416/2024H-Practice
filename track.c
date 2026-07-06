#include "track.h"
#include "delay.h"

/*
//读取指定通道 (0~7)
uint8_t Track_Read(uint8_t ch)
{

	if(ch & 0x01)	DL_GPIO_setPins(TRACK_PORT, TRACK_AD0_PIN);
	else	DL_GPIO_clearPins(TRACK_PORT, TRACK_AD0_PIN);
	if(ch & 0x02)	DL_GPIO_setPins(TRACK_PORT, TRACK_AD1_PIN);
	else	DL_GPIO_clearPins(TRACK_PORT, TRACK_AD1_PIN);
	if(ch & 0x04)	DL_GPIO_setPins(TRACK_PORT, TRACK_AD2_PIN);
	else	DL_GPIO_clearPins(TRACK_PORT, TRACK_AD2_PIN);
	delay_us(10);

	return DL_GPIO_readPins(TRACK_PORT, TRACK_OUT_PIN) ? 1 : 0;

}

//扫描所有通道
void Track_Scan(uint8_t *buf)
{
	uint8_t i;
	for(i = 0; i < 8; i++)
		buf[i] = Track_Read(i);
}
*/