#include "pid.h"

void PID_Init(PID_Controller *pid, float kp, float ki, float kd,
              float max_out, float max_int)
{
    pid->Kp = kp;  pid->Ki = ki;  pid->Kd = kd;
    pid->target = 0;  pid->output = 0;
    pid->integral = 0;  pid->prev_error = 0;
    pid->max_output = max_out;  pid->max_integral = max_int;
}

float PID_Calculate(PID_Controller *pid, float measured)
{
    float error = pid->target - measured;
    pid->integral += error;
    if (pid->integral >  pid->max_integral) pid->integral =  pid->max_integral;
    if (pid->integral < -pid->max_integral) pid->integral = -pid->max_integral;
    float derivative = error - pid->prev_error;
    pid->prev_error = error;
    pid->output = pid->Kp * error + pid->Ki * pid->integral + pid->Kd * derivative;
    if (pid->output >  pid->max_output) pid->output =  pid->max_output;
    if (pid->output < -pid->max_output) pid->output = -pid->max_output;
    return pid->output;
}

void PID_Reset(PID_Controller *pid)
{
    pid->integral = 0;
    pid->prev_error = 0;
    pid->output = 0;
}
