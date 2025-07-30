#include "../all_includes.h"

typedef struct{
    /*
    7 = motor disable
    6 = gyro
    5 = pid
    4 =  
    3 = 
    2 = 
    1 = 
    0 = 
    */
    uint8_t settings;
    uint8_t mode;
    int8_t joystick_0_x;
    int8_t joystick_0_y;
    int8_t joystick_1_x;
    int8_t joystick_1_y;

}controller_to_bot_payload;

typedef struct{
    float battery_voltage;
    float battery_current;
    float motor_current;
} bot_to_controller_payload;