#ifndef __TASK_H
#define __TASK_H

#include <stdint.h>

extern volatile uint8_t current_task;

void Task_Init(uint8_t task_num);
void Task_Run(void);
uint8_t Task_IsFinished(void);

#endif
