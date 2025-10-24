#include "../include/hardware_setup/pinout.h"
#include "../include/hardware_setup/i2c_setup.h"
#include "../include/hardware_setup/motor_offsets.h"
#include "../include/wireless_setup/structures.h"
#include "../include/all_includes.h"
#include "../include/motion_system/structures.h"
#include "../include/motion_system/kinematics.h"
#include "../include/motion_system/ramp_leg.h"
#include "../include/motion_system/ramped_kinematics.h"
#include "../include/motion_system/robot_movement.h"

#include "../include/drivers/bno055.h"

#include <nvs_flash.h>
#include "espnow_ctrl.h"
#include "espnow_utils.h"

#include "led_strip.h"

#include "driver/adc.h"
//#include "driver/i2c_master.h"
void servo_driver(void *pv);
void send_data_task(void *pv);
void led_driver(void *pv);
void current_voltage_measurements_task(void *pv);
void gyroscope_task(void *pv);

void start_esp_now();
void start_I2C();
void start_TWAI();

static uint8_t robotMacAddress[6] = {0x34, 0x85, 0x18, 0xA5, 0x74, 0x48};
static uint8_t controller_mac_address[6] = {0x68,0xB6,0xB3,0x52,0xB1,0xAC};

controller_to_bot_payload controller_to_bot = {0};


enum mode_options_enum{STAND, IK, WALK, TURN, USER, MOTORSET};


SemaphoreHandle_t mode_mutex;
uint8_t mode =0;

SemaphoreHandle_t settings_mutex;
bool pid =false;
bool gyro = false;
bool motors_disabled = true;

SemaphoreHandle_t joystick_0_x_mutex;
int8_t joystick_0_x=0;

SemaphoreHandle_t joystick_0_y_mutex;
int8_t joystick_0_y=0;

SemaphoreHandle_t joystick_1_x_mutex;
int8_t joystick_1_x=0;

SemaphoreHandle_t joystick_1_y_mutex;
int8_t joystick_1_y=0;

SemaphoreHandle_t battery_voltage_mutex;
float battery_voltage=0;

SemaphoreHandle_t battery_current_mutex;
float battery_current=0;

SemaphoreHandle_t motor_current_mutex;
float motor_current=0;


SemaphoreHandle_t led_status_mutex;
bool led_status = false;




//i2c bus config
//i2c_master_bus_config_t i2c_mst_config;

//i2c_master_bus_handle_t bus_handle;


SemaphoreHandle_t gyro_angles_mutex;
float gyro_angles[3] = {0};


// Updated callback function to handle received data
void on_receive(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len) {
    //printf("received\n");
    // printf("Received from MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
    //        recv_info->src_addr[0], recv_info->src_addr[1], recv_info->src_addr[2],
    //        recv_info->src_addr[3], recv_info->src_addr[4], recv_info->src_addr[5]);

    controller_to_bot_payload received_data;
    if (len == sizeof(received_data)) {
        memcpy(&received_data, data, sizeof(received_data));
        if(xSemaphoreTake(settings_mutex,portMAX_DELAY)){
            pid = ((received_data.settings >> 5) & 1);
            gyro = ((received_data.settings >> 6) & 1);
            motors_disabled = ((received_data.settings >> 7) & 1);
            xSemaphoreGive(settings_mutex);
        }
        if(xSemaphoreTake(mode_mutex,portMAX_DELAY)){
            mode = received_data.mode;
            xSemaphoreGive(mode_mutex);
        }
        if(xSemaphoreTake(led_status_mutex,portMAX_DELAY)){
            led_status = ((received_data.settings >> 4) & 1);
            xSemaphoreGive(led_status_mutex);
        }
    }else{
        printf("Unexpected data length: %d\n", len);
    }
}


void on_send(const uint8_t *mac_addr, esp_now_send_status_t status) {
    static bool last_transmit_state=false;
    // printf("Send to MAC: %02X:%02X:%02X:%02X:%02X:%02X, Status: %s\n",
    //        mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5],
    //        (status == ESP_NOW_SEND_SUCCESS) ? "Success" : "Fail");
    if(status == ESP_NOW_SEND_SUCCESS){
            
    }else{
        
    }
    return;
}



//task that sends controller data
void send_data_task(void *pvParameter) {
   
    while (1) {
        float last_battery_voltage = 0;
        float last_battery_current = 0;
        float last_motor_current = 0;
        if(xSemaphoreTake(battery_voltage_mutex, portMAX_DELAY)){
            last_battery_voltage = battery_voltage;
            xSemaphoreGive(battery_voltage_mutex);
        }
        
        if(xSemaphoreTake(battery_current_mutex, portMAX_DELAY)){
            last_battery_current = battery_current;
            xSemaphoreGive(battery_current_mutex);
        }

        if(xSemaphoreTake(motor_current_mutex, portMAX_DELAY)){
            last_motor_current = motor_current;
            xSemaphoreGive(motor_current_mutex);
        }
        bot_to_controller_payload current_transmission = {last_battery_voltage,last_battery_current,last_motor_current};
        esp_err_t transmit_error = esp_now_send(controller_mac_address, (uint8_t *)&current_transmission, sizeof(bot_to_controller_payload));
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}



void start_esp_now(){
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Initialize Wi-Fi in STA mode
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());

    // Initialize ESP-NOW
    ESP_ERROR_CHECK(esp_now_init());
    esp_now_register_recv_cb(on_receive);
    esp_now_register_send_cb(on_send);

    esp_now_peer_info_t peer_info = {};
    memcpy(peer_info.peer_addr, controller_mac_address, 6);
    peer_info.channel = 0;
    peer_info.encrypt = false;
    ESP_ERROR_CHECK(esp_now_add_peer(&peer_info));
}


extern "C" void app_main(void)
{
    mode_mutex = xSemaphoreCreateMutex();
    settings_mutex = xSemaphoreCreateMutex();
    joystick_0_x_mutex = xSemaphoreCreateMutex();
    joystick_0_y_mutex = xSemaphoreCreateMutex();
    joystick_1_x_mutex = xSemaphoreCreateMutex();
    joystick_1_y_mutex = xSemaphoreCreateMutex();
    battery_voltage_mutex = xSemaphoreCreateMutex();
    battery_current_mutex = xSemaphoreCreateMutex();
    motor_current_mutex = xSemaphoreCreateMutex();
    led_status_mutex = xSemaphoreCreateMutex();
    gyro_angles_mutex = xSemaphoreCreateMutex();
    start_I2C();
    start_esp_now();
    //start_TWAI();

    xTaskCreate(servo_driver,"servo_driver",4096,NULL,1,NULL);
    xTaskCreate(led_driver,"led_driver",20000,NULL,3,NULL);
    xTaskCreate(send_data_task,"send_data",4096,NULL,4,NULL);
    xTaskCreate(current_voltage_measurements_task, "current_voltage", 4096, NULL, 2, NULL);
    //xTaskCreate(gyroscope_task, "gyroscope_task", 8096, NULL, 2, NULL);
    for(;;){
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}




void led_driver(void *pv){
        /// LED strip common configuration
    led_strip_config_t led_strip_front_config = {
        .strip_gpio_num = led_strip_front,  // The GPIO that connected to the LED strip's data line
        .max_leds = 8,                 // The number of LEDs in the strip,
        .led_model = LED_MODEL_WS2812, // LED strip model, it determines the bit timing
        .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB, // The color component format is G-R-B
        .flags = {
            .invert_out = false, // don't invert the output signal
        }
    };

    /// RMT backend specific configuration
    led_strip_rmt_config_t rmt_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,    // different clock source can lead to different power consumption
        .resolution_hz = 10 * 1000 * 1000, // RMT counter clock frequency: 10MHz
        .mem_block_symbols = 64,           // the memory size of each RMT channel, in words (4 bytes)
        .flags = {
            .with_dma = false, // DMA feature is available on chips like ESP32-S3/P4
        }
    };

    /// Create the LED strip object
    led_strip_handle_t led_strip_front_handle;
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&led_strip_front_config, &rmt_config, &led_strip_front_handle));

    /// LED strip common configuration
    led_strip_config_t led_strip_back_config = {
        .strip_gpio_num = led_strip_back,  // The GPIO that connected to the LED strip's data line
        .max_leds = 8,                 // The number of LEDs in the strip,
        .led_model = LED_MODEL_WS2812, // LED strip model, it determines the bit timing
        .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB, // The color component format is G-R-B
        .flags = {
            .invert_out = false, // don't invert the output signal
        }
    };

    /// Create the LED strip object
    led_strip_handle_t led_strip_back_handle;
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&led_strip_back_config, &rmt_config, &led_strip_back_handle));

    for(uint8_t i =0; i < 8; i++){
        led_strip_set_pixel(led_strip_front_handle, i, 0, 0, 255);
        led_strip_set_pixel(led_strip_back_handle, i, 0, 0, 255);
    }

    led_strip_refresh(led_strip_front_handle);
    led_strip_refresh(led_strip_back_handle);

    static bool last_led_status = true;
    for(;;){
        bool update = false;
        if(xSemaphoreTake(led_status_mutex, portMAX_DELAY)){
            if(last_led_status != led_status){
                update = true;
                last_led_status = led_status;
            }
            xSemaphoreGive(led_status_mutex);
        }
        if(update){
            if(led_status){
                for(uint8_t i =0; i < 8; i++){
                    led_strip_set_pixel(led_strip_front_handle, i, 0, 0, 255);
                    led_strip_set_pixel(led_strip_back_handle, i, 0, 0, 255);
                }
            }else{
                for(uint8_t i =0; i < 8; i++){
                    led_strip_set_pixel(led_strip_front_handle, i, 0, 0, 0);
                    led_strip_set_pixel(led_strip_back_handle, i, 0, 0, 0);
                }
            }
            led_strip_refresh(led_strip_front_handle);
            led_strip_refresh(led_strip_back_handle);
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}


void current_voltage_measurements_task(void *pv){
    //#define current_sense_pin GPIO_NUM_13 ADC2_2
    //#define voltage_sense_pin GPIO_NUM_14 ADC2_3
    int raw_voltage_reading = 0;
    int raw_current_reading = 0; //26.4 mV/A 0-4095
    
    for(;;){
        adc2_get_raw(ADC2_CHANNEL_2, ADC_WIDTH_BIT_12, &raw_current_reading);
        adc2_get_raw(ADC2_CHANNEL_3, ADC_WIDTH_BIT_12, &raw_voltage_reading);

        if(xSemaphoreTake(battery_voltage_mutex, portMAX_DELAY)){
            battery_voltage = (raw_voltage_reading/136.0f)-2.35f;
            xSemaphoreGive(battery_voltage_mutex);
        }



        if(xSemaphoreTake(battery_current_mutex, portMAX_DELAY)){
            //2015 = measured 0 point 
            //32.76 = given 26.4 mV/A / (3.3/4095)
            battery_current = (raw_current_reading-2015)/32.76f;
            xSemaphoreGive(battery_current_mutex);
        }

        if(xSemaphoreTake(motor_current_mutex, portMAX_DELAY)){
            motor_current = 0;
            xSemaphoreGive(motor_current_mutex);
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}



void servo_driver(void *pv){
    position aCords;
    position bCords;
    position cCords;
    position dCords;

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

    //ramped_kinematics(kinematics *kinematic_driver, rampLeg *ramped_driver);
    vTaskDelay(pdMS_TO_TICKS(100));    
    ramped_kinematics a_leg_rk(&AlegK,&aLegR);
    ramped_kinematics b_leg_rk(&BlegK,&bLegR);
    ramped_kinematics c_leg_rk(&ClegK,&cLegR);
    ramped_kinematics d_leg_rk(&DlegK,&dLegR);

    position basicStand;


    basicStand.z = 130;
    basicStand.x = 0;
    basicStand.y = 0;
    
    a_leg_rk.set_stance(basicStand);
    b_leg_rk.set_stance(basicStand);
    c_leg_rk.set_stance(basicStand);
    d_leg_rk.set_stance(basicStand);

    float cycleTime = 20;
    float moveBackDistance = -50;
    float moveUpDistance =-30;

    //float xH float xFB float xLR float rot_x float rot_y float rot_z
    static position walk_position_0(basicStand.z,0,0,0,0,0);
    static position walk_position_1(basicStand.z,moveBackDistance/3,0,0,0,0);
    static position walk_position_2(basicStand.z,2*moveBackDistance/3,0,0,0,0);
    static position walk_position_3(basicStand.z+moveUpDistance,moveBackDistance,0,0,0,0);
    static position walk_position_4(basicStand.z+moveUpDistance,2*moveBackDistance/3,0,0,0,0);
    static position walk_position_5(basicStand.z+moveUpDistance,moveBackDistance/3,0,0,0,0);

    
    float stance_time = 0.2f;
    static stance walk0(&walk_position_0,&walk_position_3,&walk_position_3,&walk_position_0,stance_time);
    static stance walk1(&walk_position_1,&walk_position_4,&walk_position_4,&walk_position_1,stance_time);
    static stance walk2(&walk_position_2,&walk_position_5,&walk_position_5,&walk_position_2,stance_time);
    static stance walk3(&walk_position_3,&walk_position_0,&walk_position_0,&walk_position_3,stance_time);
    static stance walk4(&walk_position_4,&walk_position_1,&walk_position_1,&walk_position_4,stance_time);
    static stance walk5(&walk_position_5,&walk_position_2,&walk_position_2,&walk_position_5,stance_time);
    static stance *walk_forward_stances[6] = {&walk0,&walk1,&walk2,&walk3,&walk4,&walk5};
    vTaskDelay(pdMS_TO_TICKS(100));    
    
    //cycle(uint8_t cycle_length, stance **stances, bool direction);
    static cycle walk_forward(6,walk_forward_stances,true);

    static robot_movement robot_movement_test(&a_leg_rk,&b_leg_rk,&c_leg_rk,&d_leg_rk, &walk_forward);
    for(;;){
        float yAngle = 0;
        float xAngle = 0;
        float zAngle = 0;
        float xAngleV = 0;
        float yAngleV = 0;
        static uint8_t current_mode= 0;

        if(xSemaphoreTake(mode_mutex,portMAX_DELAY)){
            current_mode = mode;
            xSemaphoreGive(mode_mutex);
        }

        if(xSemaphoreTake(settings_mutex,portMAX_DELAY)){
            if(!motors_disabled){
                xSemaphoreGive(settings_mutex);
                //driver0.wake();
                //driver1.wake();
                printf("a\n");
                switch(current_mode){
                    //STAND, IK, WALK, TURN, USER, MOTORSET};
                    case(STAND):{
                        printf("b\n");
                        a_leg_rk.set_stance(basicStand);
                        b_leg_rk.set_stance(basicStand);
                        c_leg_rk.set_stance(basicStand);
                        d_leg_rk.set_stance(basicStand);
                        break;
                    }
                    case(IK):
                    printf("c\n");
                        break;
                    case(WALK):{
                        printf("d\n");
                        robot_movement_test.update();
                        break;
                    }
                    case(TURN):
                        break;
                    case(USER):
                        break;
                    case(MOTORSET):{
                        break;
                    }
                }

            }else{
                driver0.sleep();
                driver1.sleep();
                xSemaphoreGive(settings_mutex);
            }
            
        }
        vTaskDelay(pdMS_TO_TICKS(20));    

    }
}


void start_I2C(){
    // i2c_mst_config.clk_source = I2C_CLK_SRC_DEFAULT;
    // i2c_mst_config.i2c_port = I2C_NUM_0;
    // i2c_mst_config.sda_io_num = sda_pin;
    // i2c_mst_config.scl_io_num = scl_pin;
    // i2c_mst_config.glitch_ignore_cnt = 0;
    // i2c_mst_config.flags.enable_internal_pullup = true;
    // ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &bus_handle));



    i2c_config_t conf = {};
        conf.mode = I2C_MODE_MASTER; //esp32s2 will be master
        conf.master.clk_speed = 400000; //clock speed for i2c
        conf.scl_io_num = scl_pin; //scl pin
        conf.sda_io_num = sda_pin; //sda pin  
        //.scl_pullup_en = GPIO_PULLUP_ENABLE,
        //.sda_pullup_en = GPIO_PULLUP_ENABLE
    //map the config to the i2c number
    i2c_param_config(I2C_NUM_0, &conf);
    i2c_driver_install(I2C_NUM_0, conf.mode, 0, 0, 0);
}


void gyroscope_task(void *pv){
    bno055 gyro(30); //30

    for(;;){
        gyro.get_angles();
        //printf("alive\n");
        vTaskDelay(pdMS_TO_TICKS(10));
    }




}

void start_TWAI(){
    static const twai_timing_config_t t_config = TWAI_TIMING_CONFIG_100KBITS();
    static const twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    
    static const twai_general_config_t g_config = {
        .controller_id = 0,
        .mode = TWAI_MODE_NORMAL,
        .tx_io = c2_6_pin,
        .rx_io = c2_5_pin,
        .clkout_io = TWAI_IO_UNUSED,
        .bus_off_io = TWAI_IO_UNUSED,
        .tx_queue_len = 1,
        .rx_queue_len = 1,
        .alerts_enabled = TWAI_ALERT_NONE,
        .clkout_divider = 0,
        .intr_flags = 0
    };

    if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK)
    {
        assert(false);
    }

    while (twai_start() != ESP_OK)
    {
        assert(false);
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