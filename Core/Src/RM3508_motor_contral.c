#include "RM3508_motor_contral.h"
//#define speed_kp 1
//#define speed_ki 1 
//#define speed_kd 1

PIDController pidcontraller;
uint16_t pre_motor_speed = 0; // Previous motor speed, used to avoid oscillation
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
void RM3508_PID_Motor_Init(void)
{
    // Initialize the PID controller with specified gains and setpoint
    PID_init(&pidcontraller, speed_kp, speed_ki, speed_kd, 0.0f, 100.0f); // Set max_output to 100.0f as an example
    pre_motor_speed = 0; // Initialize previous motor speed
}

void RM3508_Motor_SetSpeed(uint16_t const *speed) 
{
    float Kp = speed_kp;
    float Ki = speed_ki;
    float Kd = speed_kd;
    float sp = *speed;
    float co;
    float fb = 0;
    if(pre_motor_speed != *speed) // Check if the speed has changed
    {
        pre_motor_speed = *speed; // Update the previous speed
        pid_sp_set(&pidcontraller, (float)sp); // Set the desired value (setpoint) for the PID controller
    }

    fb=(float)motor_rm3508_rx_massage().rpm; // Get the feedback value from the motor
    co = PID_compute(&pidcontraller, &fb); // Compute the control output using the PID controller
    if(co>pidcontraller.max_output)
    {
        co = pidcontraller.max_output; // Limit the output to the maximum value
    }
    else if(co<-pidcontraller.max_output)
    {
        co = -pidcontraller.max_output; // Limit the output to the minimum value
    }
    moter_rm3508_tx_massage((uint16_t)co, 0, 0, 0); // Send the control output to the motor

}