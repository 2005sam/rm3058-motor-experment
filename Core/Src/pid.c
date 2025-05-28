#include "pid.h"


//initializes the PID controller with specified gains and setpoint
  void PID_init(PIDController *pid, float Kp, float Ki, float Kd, float sp) {
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->sp = sp;
    pid->fb = 0;
    pid->pre_time = 0; 
    pid->pre_err = 0;
    pid->pre_err_integral = 0;
}

 float PID_compute(PIDController *pid, float *fd)
{
    float err = pid->sp- *fd; // Calculate the error as the difference between setpoint and feedback
    uint64_t current_time = HAL_GetTick(); 
    float dt = (current_time - pid->pre_time) *1.0e-6f;
    float d_err = (err - pid->pre_err) / dt; 
    float integral_err = pid->pre_err_integral + (err+pid->pre_err) * dt/2.0f; 
    float cop=pid->Kp*(err);
    float coi=pid->Ki*integral_err;
    float cod=pid->Kd* d_err;
    //update previous values    
    pid->pre_time = current_time;
    pid->pre_err = err;
    pid->pre_err_integral = integral_err;
    //return the PID output
    return cop + coi + cod;

}