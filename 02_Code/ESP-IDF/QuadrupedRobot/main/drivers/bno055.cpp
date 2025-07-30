#include "../../include/drivers/bno055.h"

bno055::bno055(uint8_t id){
    this->id = id;
    //send_message(id,get_status,NULL,0,true);
}




void bno055::receive_message(uint8_t message_ID){
    static float temp_angles[3] = {0};
    twai_message_t rx_message;
    esp_err_t result = twai_receive(&rx_message, pdMS_TO_TICKS(500));
    union{
        float a;
        uint8_t bytes[4];     
      } temp_union;
    if (result == ESP_OK)
    {
        //printf("Unexpected message ID: %ld\n",rx_message.identifier);
        // majority of these are not implemented as they are not needed, only the battery voltage is needed
        if (rx_message.identifier == ((id << node_id_offset) | get_status))
        {
            printf("Status: %d\n", rx_message.data[0]);
        }
        else if (rx_message.identifier == ((id << node_id_offset) | get_x))
        {
            temp_union.bytes[0] = rx_message.data[0];
            temp_union.bytes[1] = rx_message.data[1];
            temp_union.bytes[2] = rx_message.data[2];
            temp_union.bytes[3] = rx_message.data[3];
            temp_angles[0] = temp_union.a;
            printf("X: %f\n", temp_union.a);
        }
        else if (rx_message.identifier == ((id << node_id_offset) | get_y))
        {
            temp_union.bytes[0] = rx_message.data[0];
            temp_union.bytes[1] = rx_message.data[1];
            temp_union.bytes[2] = rx_message.data[2];
            temp_union.bytes[3] = rx_message.data[3];
            temp_angles[1] = temp_union.a;
            printf("y: %f\n", temp_union.a);
        }
        else if (rx_message.identifier == ((id << node_id_offset) | get_z))
        {
            temp_union.bytes[0] = rx_message.data[0];
            temp_union.bytes[1] = rx_message.data[1];
            temp_union.bytes[2] = rx_message.data[2];
            temp_union.bytes[3] = rx_message.data[3];
            temp_angles[2] = temp_union.a;
            printf("z: %f\n", temp_union.a);
        }
        else{
            printf("Unexpected message ID: %ld\n",rx_message.identifier);
        }
        // if(xSemaphoreTake(gyro_angles_mutex, portMAX_DELAY)){
        //     memcpy(gyro_angles,temp_angles,sizeof(temp_angles));
        //     xSemaphoreGive(gyro_angles_mutex);
        // }
    }
    else
    {
        printf("No message received\n");
    }
}


void bno055::send_message(const uint8_t message_ID, const uint8_t *message_contents, uint8_t message_length, const bool data_returned){
    // max data size is 8 bytes
    if (message_length > 8)
    {
        return;
    }
    if (message_contents == nullptr)
    {
        message_length = 0;
    }

    // setup can node + message ID with some bit manipulation to ensure the messageID is the least sigificant node_id_offset bits
    uint32_t can_ID = (id << node_id_offset) | (message_ID & 0x1F);

    // setup the message to transmit
    twai_message_t tx_message;
    tx_message.identifier = can_ID;
    (void)memcpy(tx_message.data, message_contents, message_length); // copy the message data
    tx_message.extd = 0;                                       // not a extended frame
    tx_message.rtr = 0;  // no remote transmission request
    tx_message.data_length_code = message_length;              // set the number of bytes in the message


    esp_err_t result = twai_transmit(&tx_message, pdMS_TO_TICKS(200));


    twai_message_t rx_message;
    if(data_returned){
        while(twai_receive(&rx_message,0)== ESP_OK){
            
        }
    }
    // if the data was sent successfully, and a response is expected
    if (data_returned && (result == ESP_OK))
    {
        receive_message(message_ID);
    }else{
        printf("Message not sent\n");
    }
}

void bno055::get_angles(){
    send_message(get_status,nullptr,0,true); 
    send_message(get_x,nullptr,0,true); 
    send_message(get_y,nullptr,0,true); 
    send_message(get_z,nullptr,0,true); 
}
