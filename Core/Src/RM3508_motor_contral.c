#include "RM3508_motor_contral.h"
//#define speed_kp 1
//#define speed_ki 1 
//#define speed_kd 1
#define angle_kp 0.002f // Proportional gain for angle control
#define angle_ki 0.0005f // Integral gain for angle control
#define angle_kd 0.0f // Derivative gain for angle control

PIDController pidcontraller;
PIDController angle_pid_contraller;
int16_t pre_motor_speed = 0; // Previous motor speed, used to avoid oscillation
//warring:this function is only used to regulating PID,plase delete it in the final version
/***********************************************************************************************/
float speed_kp = 100.0f; // Proportional gain
float speed_ki = 100.0f; // Integral gain
float speed_kd = 20.0f; // Derivative gain
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
    PID_init(&pidcontraller, 2.0, 1.0,1.0,0.0f, 0.0f, 16384.0f,0.0f,0.0f,0,0); // Set max_output to 100.0f as an example
    PID_init(&angle_pid_contraller, angle_kp, angle_ki, angle_kd,0.0f, 0.0f, 10.0f,0.3,1.0,0.0,1.0f); // Set max_output to 100.0f as an example
    pre_motor_speed = 0; // Initialize previous motor speed
}

void RM3508_Motor_SetSpeed(int16_t const *speed) 
{
    float sp = (float)*speed;
    float co;
    float fb = 0;


    //update setspeed if speed changed
    if(pre_motor_speed != *speed) 
    {
        pre_motor_speed = *speed; 
        pid_sp_set(&pidcontraller, (float)sp);
    }


    //comput co and tx to moter
    fb=(float)motor_rm3508_get_rx_date(0).rpm; 
    co = PID_compute(&pidcontraller, &fb); 
    moter_rm3508_tx_massage((int16_t)co, 0, 0, 0);

}


// used to set control the angle of the motor
//return the speed should be set to the motor
float RM3508_Motor_SetAngle(float angle) 
{
    float Kp = angle_kp;
    float Ki = angle_ki;
    float Kd = angle_kd;
    float sp = angle;
    float co;
    float fb = 0;

    pid_sp_set(&angle_pid_contraller, sp); // Set the desired value (setpoint) for the PID controller
    fb = motor_rm3508_rx_massage(0).angle; // Get the feedback value from the motor
    co = PID_compute(&angle_pid_contraller, &fb); // Compute the control output using the PID controller
		co=10.0f;
    RM3508_Motor_SetSpeed((int16_t *)&co); // Set the speed based on the control output
    return co; // Return the control output (speed) to be set to the motor
}