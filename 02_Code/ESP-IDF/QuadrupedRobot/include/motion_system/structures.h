#ifndef __motion_system_structures__
#define __motion_system_structures__
#include "../all_includes.h"

struct position{
    float x;
    float y;
    float z;
    float pitch;
    float roll;
    float yaw;
};

struct all_motor_angles{
    float A_hip;
    float A_knee;
    float A_ankle;
    float B_hip;
    float B_knee;
    float B_ankle;
    float C_hip;
    float C_knee;
    float C_ankle;
    float D_hip;
    float D_knee;
    float D_ankle;
};

#endif 