
#ifndef RM3508_MOTOR_CONTRAL_H
#define RM3508_MOTOR_CONTRAL_H
#include "stm32f4xx_hal.h"
#include "MotorRM3508Drive.h"
#include "pid.h"

void RM3508_Motor_SetSpeed(int16_t const * speed);



#endif