#include "../include/hardware_setup/pinout.h"
#include "../include/hardware_setup/i2c_setup.h"
#include "../include/hardware_setup/motor_offsets.h"
#include "../include/wireless_setup/structures.h"
#include "../include/all_includes.h"
#include "../include/motion_system/structures.h"
#include "../include/motion_system/kinematics.h"
#include "../include/motion_system/ramp_leg.h"

void servo_driver(void *pv);


controller_to_bot_payload controller_to_bot = {0};


single_leg aCords;
single_leg bCords;
single_leg cCords;
single_leg dCords;

all_motor_angles activeOffsets={(float)A_HIP_OFFSET,(float)A_KNEE_OFFSET,(float)A_ANKLE_OFFSET,
                            (float)B_HIP_OFFSET,(float)B_KNEE_OFFSET,(float)B_ANKLE_OFFSET,
                            (float)C_HIP_OFFSET,(float)C_KNEE_OFFSET,(float)C_ANKLE_OFFSET,
                            (float)D_HIP_OFFSET,(float)D_KNEE_OFFSET,(float)D_ANKLE_OFFSET};

    pca9685 driver0((uint8_t)pca9685_address_0);
    pca9685 driver1((uint8_t)pca9685_address_1);
    // motor(pca9685* pwmV, uint8_t motorV, uint16_t LlimitV, uint16_t HlimitV, bool directionV, float* offsetV);
    motor aHipM(&driver0, (uint8_t)aHip, 45, 135, true, &(activeOffsets.A_hip));
    motor aKneeM(&driver0, (uint8_t)aKnee, 0, 180, false, &(activeOffsets.A_knee));
    motor aAnkleM(&driver0, (uint8_t)aAnkle, 0, 180, true, &(activeOffsets.A_ankle));

    motor bHipM(&driver0, bHip, 45, 135, false, &(activeOffsets.B_hip));
    motor bKneeM(&driver0, bKnee, 0, 180, true, &(activeOffsets.B_knee));
    motor bAnkleM(&driver0, bAnkle, 0, 180, false, &(activeOffsets.B_ankle));

    motor cHipM(&driver1, cHip, 45, 135, false, &(activeOffsets.C_hip));
    motor cKneeM(&driver1, cKnee, 0, 180, false, &(activeOffsets.C_knee));
    motor cAnkleM(&driver1, cAnkle, 0, 180, true, &(activeOffsets.C_ankle));

    motor dHipM(&driver1, dHip, 45, 135, true, &(activeOffsets.D_hip));
    motor dKneeM(&driver1, dKnee, 0, 180, true, &(activeOffsets.D_knee));
    motor dAnkleM(&driver1, dAnkle, 0, 180, false, &(activeOffsets.D_ankle));

    leg Aleg(&aHipM, &aKneeM, &aAnkleM, 'A');
    leg Bleg(&bHipM, &bKneeM, &bAnkleM, 'B');
    leg Cleg(&cHipM, &cKneeM, &cAnkleM, 'C');
    leg Dleg(&dHipM, &dKneeM, &dAnkleM, 'D');

    kinematics AlegK(&Aleg);
    kinematics BlegK(&Bleg);
    kinematics ClegK(&Cleg);
    kinematics DlegK(&Dleg);

    rampLeg aLegR(aHip);
    rampLeg bLegR(bHip);
    rampLeg cLegR(cHip);
    rampLeg dLegR(dHip);


single_leg basicStand;
    

void start_I2C(){
    i2c_config_t i2cConfig = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = sda_pin,
        .scl_io_num = scl_pin,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master{
            .clk_speed = i2c_freq
        }
    };

    //map the config to the i2c number
    i2c_param_config(i2c_bus_number, &i2cConfig);
    i2c_driver_install(i2c_bus_number, i2cConfig.mode, 0, 0, 0);
}



extern "C" void app_main(void)
{
    start_I2C();
    xTaskCreate(servo_driver,"servo_driver",10000,NULL,1,NULL);
    for(;;){
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}



void servo_driver(void *pv){
    
    basicStand.xH = 130;
    basicStand.xFB = 0;
    basicStand.xLR = 0;
    

    AlegK.mainKinematics(basicStand);
    BlegK.mainKinematics(basicStand);
    ClegK.mainKinematics(basicStand);
    DlegK.mainKinematics(basicStand);

    float cycleTime = 50;
    float moveBackDistance = -50;
    float moveUpDistance =-30;

    
    for(;;){
        float yAngle = 0;
        float xAngle = 0;
        float zAngle = 0;
        float xAngleV = 0;
        float yAngleV = 0;
        static uint8_t oldState = 0;
        vTaskDelay(pdMS_TO_TICKS(1));
        // if(cycle == 0 && test.is_complete()){
        //     test.set_target(-90, 5);
        //     cycle = 1;
        // }else if(cycle == 1 && test.is_complete()){
        //     test.set_target(-180, 5);
        //     cycle = 0;
        // }
        //ESP_LOGI("interpolation","current position: %f",test.get_position());

    

    }
}















































// void start_TWAI(){
//     static const twai_timing_config_t timing_config = TWAI_TIMING_CONFIG_50KBITS();
//     static const twai_filter_config_t filter_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
//     static const twai_general_config_t general_config = {
//         .mode = TWAI_MODE_NORMAL,
//         .tx_io = can_TX_pin,
//         .rx_io = can_RX_pin,
//         .clkout_io = TWAI_IO_UNUSED,
//         .bus_off_io = TWAI_IO_UNUSED,
//         .tx_queue_len = 1,
//         .rx_queue_len = 1,
//         .alerts_enabled = TWAI_ALERT_NONE,
//         .clkout_divider = 0
//     };
  
//     if (twai_driver_install(&general_config, &timing_config, &filter_config) == ESP_OK) {
//         ESP_LOGI("can", "TWAI driver installed");
//     } else {
//         ESP_LOGE("can", "Failed to install TWAI driver");
//     }

//     if (twai_start() != ESP_OK) {
//         ESP_LOGI("can", "Failed to start TWAI driver");
//         vTaskDelay(pdMS_TO_TICKS(1000));
//         esp_restart();
//     }
// }