#ifndef __pid__
#define __pid__
#include "esp_timer.h"


class pid{
    private:
        float kp;
        float ki;
        float kd;
        float integral;
        float last_error;
        uint64_t last_time;
        float output_max;
        float output_min;
        float current_output;
    public:
        pid(float kp, float ki, float kd, float output_min, float output_max);
        float calculate(float setpoint);
};
#endif