#include "../include/motion_system/motor.h"
motor::motor(pca9685* pwmV, uint8_t motorV, uint16_t LlimitV, uint16_t HlimitV, bool directionV, float* offsetV){
    pwm = pwmV;
    motorC = motorV;
    Llimit = LlimitV;
    Hlimit = HlimitV;
    direction = directionV;
    offset = offsetV;
}

void motor::set_degree(float nDegree){
    if(nDegree+*offset > Hlimit){
      nDegree = Hlimit-*offset;
    }else if(nDegree+*offset < Llimit){
      nDegree = Llimit-*offset; 
    }else{
        nDegree = nDegree + *offset;
    }
    if(direction){
        pwm->writeValue(motorC, (map(nDegree,0,180,servo_min,servo_max)));
    }else{
        pwm->writeValue(motorC, (map(nDegree,180,0,servo_min,servo_max)));
    }
}

uint8_t motor::getMotor(){
    return motorC;
}