#include "../include/pid.h"


pid::pid(float kp, float ki, float kd, float output_min, float output_max){
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
    this->integral = 0;
    this->last_error = 0;
    this->last_time = esp_timer_get_time();

    this->output_min = output_min;
    this->output_max = output_max;
    
    this->current_output = 0;
}


float pid::calculate(float setpoint){
    uint64_t current_time = esp_timer_get_time();
    float delta_time = (current_time - last_time)/1000000.0;
    
    float error = setpoint - current_output;
    integral += error * delta_time;


    float derivative = (error - last_error) / delta_time;
    float output = kp * error + ki * integral + kd * derivative;
    last_error = error;
    last_time = current_time;
    if(output > output_max){
        output = output_max;
    }else if(output < output_min){
        output = output_min;
    }
    current_output = output;
    return output;
}
