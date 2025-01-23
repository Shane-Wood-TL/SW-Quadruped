#include "../all_includes.h"

struct controller_to_bot_payload{
    uint8_t settings;
    //0 = estop
    //1 = gyro_enable
    //2 = pid_enable
    uint8_t state;
    int8_t j1_x;
    int8_t j1_y;
    int8_t j2_x;
    int8_t j2_y;
};

struct bot_to_controller_payload{
    uint16_t current_usage;
    uint16_t battery_voltage;
};