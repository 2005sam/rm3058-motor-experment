#include "RM3508_motor_contral.h"
//#define speed_kp 1
//#define speed_ki 1 
//#define speed_kd 1

//warring:this function is only used to regulating PID,plase delete it in the final version
/***********************************************************************************************/
float speed_kp = 1.0f; // Proportional gain
float speed_ki = 1.0f; // Integral gain
float speed_kd = 1.0f; // Derivative gain
void receive_date(float date,char flag)
{
    if(flag == 0xA1)
    {
     speed_kp = date;
    }
    else if(flag == 0xA2)
    {
        speed_ki = date;
    }
    else if(flag == 0xA3)
    {
       speed_kd = date;
    }

}
/***********************************************************************************************/
void RM3508_Motor_SetSpeed(uint16_t const *speed) 
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