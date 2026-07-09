#ifndef __TASK_H
#define __TASK_H

#include <stdint.h>

void Task_Init(uint8_t task_num);
void Task_Run(void);
uint8_t Task_IsFinished(void);

#endif
