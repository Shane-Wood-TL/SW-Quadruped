#ifndef __pca9685__
#define __pca9685__
#include "../all_includes.h"
class pca9685{
    private:
        uint8_t address;
        esp_err_t send_command(uint8_t register_to_change, uint8_t value);
    public:
        pca9685(uint8_t address_value);
        void writeValue(uint8_t motor, uint16_t value);
        void sleep();
        void wake();
};

#endif