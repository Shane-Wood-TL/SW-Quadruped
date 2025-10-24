#ifndef __pinout__
#define __pinout__

#include "../all_includes.h"


#define usbDataMinus GPIO_NUM_19
#define usbDataPlus GPIO_NUM_20

#define sda_pin GPIO_NUM_4
#define scl_pin GPIO_NUM_5

#define c0_5_pin GPIO_NUM_6
#define c0_6_pin GPIO_NUM_7
#define c0_7_pin GPIO_NUM_15
#define c0_8_pin GPIO_NUM_16
#define c0_9_pin GPIO_NUM_17

#define c1_5_pin GPIO_NUM_18
#define c1_6_pin GPIO_NUM_8
#define c1_7_pin GPIO_NUM_9
#define c1_8_pin GPIO_NUM_10
#define c1_9_pin GPIO_NUM_11

#define current_sense_pin GPIO_NUM_13
#define voltage_sense_pin GPIO_NUM_14

#define c2_5_pin GPIO_NUM_1
#define c2_6_pin GPIO_NUM_2
#define c2_7_pin GPIO_NUM_42
#define c2_8_pin GPIO_NUM_41
#define c2_9_pin GPIO_NUM_40

#define c3_5_pin GPIO_NUM_39
#define c3_6_pin GPIO_NUM_38
#define c3_7_pin GPIO_NUM_37
#define c3_8_pin GPIO_NUM_36
#define c3_9_pin GPIO_NUM_35

#define can_TX_pin GPIO_NUM_48
#define can_RX_pin GPIO_NUM_47


#define led_strip_front c0_6_pin
#define led_strip_back c3_6_pin


#endif



/*Controller
#define x0_pin GPIO_NUM_4
#define y0_pin GPIO_NUM_5
#define z0_pin GPIO_NUM_6

#define x1_pin GPIO_NUM_7
#define y1_pin GPIO_NUM_15
#define z1_pin GPIO_NUM_16

#define btn_0 GPIO_NUM_17
#define btn_1 GPIO_NUM_18

#define swA_0 GPIO_NUM_8
#define swA_1 GPIO_NUM_9

#define swB_0 GPIO_NUM_10
#define swB_1 GPIO_NUM_38

#define swC_0 GPIO_NUM_37
#define swC_1 GPIO_NUM_36

#define swD_0 GPIO_NUM_35
#define swD_1 GPIO_NUM_48

#define swE_0 GPIO_NUM_47
#define swE_1 GPIO_NUM_21

#define mosi_pin GPIO_NUM_11
#define sclk_pin GPIO_NUM_13
#define dataCommand_pin GPIO_NUM_14

#define d0_CS GPIO_NUM_1
#define d0_RS GPIO_NUM_41

#define d1_CS GPIO_NUM_2
#define d1_RS GPIO_NUM_40

#define d2_CS GPIO_NUM_42
#define d2_RS GPIO_NUM_39
*/