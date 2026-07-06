#ifndef __TRACKING_H
#define __TRACKING_H

#include "ti_msp_dl_config.h"

uint8_t Track_Read(uint8_t ch); //读取指定通道 (0~7)
void Track_Scan(uint8_t *buf); //扫描所有通道，结果存入buf

#endif
