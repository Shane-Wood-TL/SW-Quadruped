#ifndef __i2cSetup__
#define __i2cSetup__

#include "../all_includes.h"
//i2c setup
#define i2c_freq 400000   // Frequency of I2C (400 kHz)
#define i2c_bus_number I2C_NUM_0    // I2C port number

#define pca9685_address_0 0x60
#define pca9685_address_1 0x7C
#define bno055_address 0x28
#define ssd1306_address 0x3C


#define ssd1306_vertical_resolution 32           //32 pixels tall
#define ssd1306_horizontal_resolution 128    //128 pixels across
#define ssd1306_vertical_page_count ssd1306_vertical_resolution/8 //each page is a vertical 8 bits
#endif