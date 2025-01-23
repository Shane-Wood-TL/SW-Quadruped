#include "../include/all_includes.h"
#include "../include/motion_system/motor.h"
#include "../include/motion_system/singleCycleClass.h"
#include "../include/motion_system/kinematicsClass.h"
#include "../include/motion_system/legClass.h"
#include "../include/motion_system/rampLegClass.h"
#include "../include/motion_system/movementCyclesClass.h"
#include "../include/motion_system/cycleControlClass.h"
#include "../include/hardware_setup/motor_offsets.h"
#include "../include/function_declarations.h"
void servo_driver(void *pv);


PayloadStruct payload = {0};


Cords aCords;
Cords bCords;
Cords cCords;
Cords dCords;

singleCycle activeOffsets(A_HIP_OFFSET,A_KNEE_OFFSET,A_ANKLE_OFFSET,
                            B_HIP_OFFSET,B_KNEE_OFFSET,B_ANKLE_OFFSET,
                            C_HIP_OFFSET,C_KNEE_OFFSET,C_ANKLE_OFFSET,
                            D_HIP_OFFSET,D_KNEE_OFFSET,D_ANKLE_OFFSET);

    pca9685 driver0((uint8_t)pca9685_address_0);
    pca9685 driver1((uint8_t)pca9685_address_1);

    motor aHipM(&driver0, aHip, 45, 135, true, &(activeOffsets.legPositions[0]));
    motor aKneeM(&driver0, aKnee, 0, 180, false, &(activeOffsets.legPositions[1]));
    motor aAnkleM(&driver0, aAnkle, 0, 180, true, &(activeOffsets.legPositions[2]));

    motor bHipM(&driver0, bHip, 45, 135, false, &(activeOffsets.legPositions[3]));
    motor bKneeM(&driver0, bKnee, 0, 180, true, &(activeOffsets.legPositions[4]));
    motor bAnkleM(&driver0, bAnkle, 0, 180, false, &(activeOffsets.legPositions[5]));

    motor cHipM(&driver1, cHip, 45, 135, false, &(activeOffsets.legPositions[6]));
    motor cKneeM(&driver1, cKnee, 0, 180, false, &(activeOffsets.legPositions[7]));
    motor cAnkleM(&driver1, cAnkle, 0, 180, true, &(activeOffsets.legPositions[8]));

    motor dHipM(&driver1, dHip, 45, 135, true, &(activeOffsets.legPositions[9]));
    motor dKneeM(&driver1, dKnee, 0, 180, true, &(activeOffsets.legPositions[10]));
    motor dAnkleM(&driver1, dAnkle, 0, 180, false, &(activeOffsets.legPositions[11]));

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


Cords basicStand;
    

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

    singleCycle walking0(0,0,moveBackDistance/3, // moving back
                    moveUpDistance,0,2*moveBackDistance/3, //returning to 0
                    moveUpDistance,0,2*moveBackDistance/3, //returning to 0
                    0,0,moveBackDistance/3); // moving back


    singleCycle walking1(0,0,2*moveBackDistance/3, // moving back
                        moveUpDistance,0,moveBackDistance/3, //returning to 0
                        moveUpDistance,0,moveBackDistance/3,+//returning to 0
                        0,0,2*moveBackDistance/3); // moving back


    singleCycle walking2(0,0,moveBackDistance, //moved back
                        0,0,0, //back at 0
                        0,0,0, //back at 0
                        0,0,moveBackDistance); //moved back


    singleCycle walking3(moveUpDistance,0,2*moveBackDistance/3, //returning to 0
                        0,0,moveBackDistance/3, //moving back
                        0,0,moveBackDistance/3, //moving back
                        moveUpDistance,0,2*moveBackDistance/3); //returning to 0

    singleCycle walking4(moveUpDistance,0,moveBackDistance/3, //returning to 0
                        0,0,2*moveBackDistance/3, // moving back
                        0,0,2*moveBackDistance/3, // moving back
                        moveUpDistance,0,moveBackDistance/3); //returning to 0

    singleCycle walking5(0,0,0, //back at 0
                        0,0,moveBackDistance, //moved back
                        0,0,moveBackDistance, //moved back
                        0,0,0); //back at 0




    //switch to size of instead of a strict n value
    singleCycle walking[] = {walking0,walking1,walking2,walking3,walking4,walking5};
    movementCycles walkForward(6, false,true,3,walking);

    Cords aCords;
    Cords bCords;
    Cords cCords;
    Cords dCords;

    Cords AcurrentPosition;
    Cords BcurrentPosition;
    Cords CcurrentPosition;
    Cords DcurrentPosition;
    AcurrentPosition.xH=130;
    BcurrentPosition.xH=130;
    CcurrentPosition.xH=130;
    DcurrentPosition.xH=130;
    cycleControl walkForwardCycle(&walkForward, 
                                &aLegR,&bLegR,&cLegR,&dLegR,
                                &AlegK,&BlegK,&ClegK,&DlegK,
                                &aCords,&bCords,&cCords,&dCords);


    singleCycle recievedPositions;

    // interpolation test(0);
    // uint8_t cycle =0;
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

        walkForwardCycle.continueCycle(AcurrentPosition,BcurrentPosition,CcurrentPosition,DcurrentPosition);
        // AlegK.mainKinematics(basicStand);
        // BlegK.mainKinematics(basicStand);
        // ClegK.mainKinematics(basicStand);
        // DlegK.mainKinematics(basicStand);


        if(payload.eStop != 1){
            wakeup_9();
            if(oldState !=payload.state){
                // aLegR.setCycle(0);
                // bLegR.setCycle(3);
                // cLegR.setCycle(3);
                // dLegR.setCycle(0);
            }
            switch (payload.state) {
                case 0:{ //standing
                    standing_0();
                break;
                }
                case 1:{ //IK mode
                    IK_1(0,yAngle,zAngle);
                break;
                }
                case 2:{//FWalk
                    FWalk_2(xAngleV,yAngleV);
                break;
                }
                case 3:{ //
                    FTurn_3(xAngleV,yAngleV);
                break;
                }
                case 4:{ //user
                    User_4(xAngleV,yAngleV);
                break;
                } 
                case 5:{ //used to install new motors
                    break;
                }
                case 6:{ //motor offset setup
                    break;
                }
                case 7:{
                    break;
                }
            default:
                break;
                }
            }else{
                Default_9(); //turns off motors
            }
        oldState = payload.state;

    }
}















































void start_TWAI(){
    static const twai_timing_config_t timing_config = TWAI_TIMING_CONFIG_50KBITS();
    static const twai_filter_config_t filter_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    static const twai_general_config_t general_config = {
        .mode = TWAI_MODE_NORMAL,
        .tx_io = can_TX_pin,
        .rx_io = can_RX_pin,
        .clkout_io = TWAI_IO_UNUSED,
        .bus_off_io = TWAI_IO_UNUSED,
        .tx_queue_len = 1,
        .rx_queue_len = 1,
        .alerts_enabled = TWAI_ALERT_NONE,
        .clkout_divider = 0
    };
  
    if (twai_driver_install(&general_config, &timing_config, &filter_config) == ESP_OK) {
        ESP_LOGI("can", "TWAI driver installed");
    } else {
        ESP_LOGE("can", "Failed to install TWAI driver");
    }

    if (twai_start() != ESP_OK) {
        ESP_LOGI("can", "Failed to start TWAI driver");
        vTaskDelay(pdMS_TO_TICKS(1000));
        esp_restart();
    }
}