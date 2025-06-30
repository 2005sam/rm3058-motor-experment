#include "pid.h"


//initializes the PID controller with specified gains and setpoint
  void PID_init(PIDController *pid, float Kp, float Ki, float Kd,float fd, float fp, float max_output,float ki_start_err,float deadband)
{
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->fd = fd;
    pid->fp = fp;
    pid->fb = 0;
    pid->pre_time = 0; 
    pid->pre_err = 0;
    pid->pre_err_integral = 0;
    pid->max_output = max_output; 
    pid->ki_start_err= ki_start_err;
    pid->deadband=deadband;
}

//update setpoint for the PID controller
void pid_sp_set(PIDController *pid, float sp)
{
    pid->sp = sp;
}
// Computes the PID control output based on the current feedback value
 float PID_compute(PIDController *pid, float *fd)
{
    // Check if the previous time is zero, indicating this is the first call
    if(pid->pre_time ==0){
        pid->pre_time = HAL_GetTick();
    }
    uint64_t current_time = HAL_GetTick(); 
    
    
    // Calculate the error between the setpoint and feedback value
    float err = pid->sp- *fd;
    
    
    // Check if the error is within the deadband range
    if(err > pid->deadband || err < -pid->deadband) {
        err = 0; 
    }
    
    
    //check if the error is greater than the start error for Ki
    char ki_flag= 0;
    if(err> pid->ki_start_err || err < -pid->ki_start_err) {
        ki_flag = 1;
    }
    

    //compute the integral and derivative errors
    float dt = (current_time - pid->pre_time) *1.0e-6f;
    float d_err = (err - pid->pre_err) / dt;
    float integral_err = pid->pre_err_integral + (err+pid->pre_err) * dt/2.0f; 
    

    //compute the cop, coi, and cod values
    // cop: proportional term, coi: integral term, cod: derivative term 
    float cop=pid->Kp*(err);
    float coi=ki_flag*pid->Ki*integral_err;
    float cod=pid->Kd* d_err;
    float co= cop + coi + cod;
      
    //compute the feedforward term
    if(pid->sp ==0){
        pid->sp = pid->pre_target;
    }
    float cof= pid->fd * (pid->sp - pid->pre_target) / dt+(pid->sp - pid->pre_target) * pid->fp;

    
    //update previous values    
    pid->pre_time = current_time;
    pid->pre_err = err;
    pid->pre_err_integral = integral_err;
    pid->pre_target = pid->sp;
    
    
    // Limit the control output to the maximum and minimum values
    if(co > pid->max_output) {
        co = pid->max_output; // Limit the output to the maximum value
    } else if(co < -pid->max_output) {
        co = -pid->max_output; // Limit the output to the minimum value
    }


    return co+ cof; // Return the total control output including feedforward

}