#ifndef __bno055__
#define __bno055__
#include "../all_includes.h"


#define get_status 0
#define get_x 1
#define get_y 2
#define get_z 3

extern SemaphoreHandle_t gyro_angles_mutex;
extern float gyro_angles[3];


class bno055{
    private:
        uint8_t id;
        void receive_message(uint8_t message_ID);
        void send_message(const uint8_t message_ID, const uint8_t *message_contents, uint8_t message_length, const bool data_returned);
    public:
        bno055(uint8_t id);
        void get_angles();
};

#endif