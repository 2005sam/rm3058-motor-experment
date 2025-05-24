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
    uint64_t pre_time;//last pid calculation time
    float pre_err;// Previous error value
    float pre_err_integral;// Previous integral of error
} PIDController;
#endif
