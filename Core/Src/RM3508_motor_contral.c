#include "RM3508_motor_contral.h"
#define speed_kp 1
#define speed_ki 1 
#define speed_kd 1
void RM3508_Motor_SetSpeed(int16_t const *speed) 
{
    PIDController pidcontraller;
    float Kp = speed_kp;
    float Ki = speed_ki;
    float Kd = speed_kd;
    float sp = *speed;
    float co;
    float fb = 0;
    fb= motor_rm3508_get_rx_date().rpm; // Get the current speed from the motor feedback
    co=PID_contral(&pidcontraller, Kp, Ki, Kd, sp,fb);
    moter_rm3508_tx_massage(co, 0,0,0);

}