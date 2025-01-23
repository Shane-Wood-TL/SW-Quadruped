#include "../include/all_includes.h"



pca9685::pca9685(uint8_t address_value){
    address = address_value;
    sleep();
    send_command(0xFE, 121); //50hz
    wake();
    send_command(0x01,0x04); //totem-pole mode
}


void pca9685::wake(){
    send_command(0x00, 0x00); //wake
}

void pca9685::sleep(){
    send_command(0x00, 0x10); //sleep
}

esp_err_t pca9685::send_command(uint8_t register_to_change, uint8_t value)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create(); // this is the current i2c "event" that is occuring
    // cmd stores all commands before executing them

    i2c_master_start(cmd); // start the event
    // set up writing the the correct address (master is writing to the address of the 1306)
    // i2c_master_write_byte(cmd, (ESP_SLAVE_ADDR << 1) | I2C_MASTER_WRITE, ACK_EN);
    i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_WRITE, true);

    i2c_master_write_byte(cmd, register_to_change, true);    // if data == 0, then command is being sent
    i2c_master_write_byte(cmd, value, true); // set the command that was given in parameter

    i2c_master_stop(cmd); // stop the transaction

    // this line actually does all of the lines above with a timeout period
    esp_err_t ret = i2c_master_cmd_begin(i2c_bus_number, cmd, 1000 / portTICK_PERIOD_MS);

    i2c_cmd_link_delete(cmd); // delete the link/event
    return ret;               // return if the writing was sucessful
}

void pca9685::writeValue(uint8_t pin, uint16_t value){ //mircoseconds
    uint8_t on_low_byte = 0x06 + (4 * pin);
    uint8_t on_high_byte = 0x07 + (4 * pin);
    uint8_t off_low_byte = 0x08 + (4 * pin);
    uint8_t off_high_byte = 0x09 + (4 * pin);

    //12 bit total time (4096). sets time that is on (likely 0) and off (likely some point in the future)
    uint16_t offTime = (uint16_t)((value * 4096) / 20000);
    
    if(offTime > 4096){
        offTime = 4096;
    }

    send_command(on_low_byte,0 & 0xFF);
    send_command(on_high_byte,(0 >>8) & 0xFF);

    send_command(off_low_byte,offTime & 0xFF);
    send_command(off_high_byte,(offTime >>8) & 0xFF);
}