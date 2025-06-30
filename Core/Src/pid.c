#include "pid.h"


//initializes the PID controller with specified gains and setpoint
  void PID_init(PIDController *pid, float Kp, float Ki, float Kd, float sp, float max_output,float ki_start_err,float deadband) {
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->sp = sp;
    pid->fb = 0;
    pid->pre_time = 0; 
    pid->pre_err = 0;
    pid->pre_err_integral = 0;
    pid->max_output = max_output; // Set the maximum output value, if needed
    pid->ki_start_err= ki_start_err; // Set the start error for Ki to avoid integral windup at the beginning
    pid->deadband=deadband;
}

void pid_sp_set(PIDController *pid, float sp)
{
    pid->sp = sp; // Set the desired value (setpoint) for the PID controller
}

 float PID_compute(PIDController *pid, float *fd)
{
    if(pid->pre_time == 0) { // If this is the first call, initialize pre_time
        pid->pre_time = HAL_GetTick(); // Get the current time in milliseconds
    }
    float err = pid->sp- *fd; // Calculate the error as the difference between setpoint and feedback
    if(err > pid->deadband || err < -pid->deadband) { // Check if the error is outside the deadband
        err = 0; // If within deadband, set error to zero
    }
    uint64_t current_time = HAL_GetTick(); 
    float dt = (current_time - pid->pre_time) *1.0e-6f;
    float d_err = (err - pid->pre_err) / dt;
    char ki_flag= 0; // Flag to indicate if Ki should be applied 
    if(err> pid->ki_start_err || err < -pid->ki_start_err) { // Check if the error is greater than the start error for Ki
        ki_flag = 1; // Set the flag to apply Ki
    }

    float integral_err = pid->pre_err_integral + (err+pid->pre_err) * dt/2.0f; 
    float cop=pid->Kp*(err);
    float coi=ki_flag*pid->Ki*integral_err;
    float cod=pid->Kd* d_err;
    //update previous values    
    pid->pre_time = current_time;
    pid->pre_err = err;
    pid->pre_err_integral = integral_err;
    float co= cop + coi + cod; // Calculate the control output as the sum of proportional, integral, and derivative terms
    if(co > pid->max_output) {
        co = pid->max_output; // Limit the output to the maximum value
    } else if(co < -pid->max_output) {
        co = -pid->max_output; // Limit the output to the minimum value
    }
    return co; // Return the control output

}