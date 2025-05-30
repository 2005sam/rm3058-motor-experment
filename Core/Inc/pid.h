#ifndef PID_H
#define PID_H
#include <stm32f4xx_hal.h>
#include "stdint.h"

typedef struct 
{
    float Kp;  // Proportional gain
    float Ki;  // Integral gain
    float Kd;  // Derivative gain
    float sp;  // Desired value
    float fb;  // Feedback value
    uint64_t pre_time;//last pid calculation time
    float pre_err;// Previous error value
    float pre_err_integral;// Previous integral of error
    float max_output; // Maximum output value (optional, can be used for limiting)
} PIDController;

void pid_sp_set(PIDController *pid, float sp);
void PID_init(PIDController *pid, float Kp, float Ki, float Kd, float sp, float max_output);
float PID_compute(PIDController *pid, float *fd);
#endif