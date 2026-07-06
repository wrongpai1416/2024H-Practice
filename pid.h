#ifndef __PID_H
#define __PID_H

#include <stdint.h>

typedef struct {
    float Kp, Ki, Kd;
    float target;
    float output;
    float integral, prev_error;
    float max_output, max_integral;
} PID_Controller;

void PID_Init(PID_Controller *pid, float kp, float ki, float kd,
              float max_out, float max_int);
float PID_Calculate(PID_Controller *pid, float measured);
void PID_Reset(PID_Controller *pid);

#endif
