typedef struct position{
  float x;
  float y;
  float z;
  float pitch;
  float roll;
  float yaw;
} position;

typedef struct time_position{
  position target_position;
  float time_to_move;
} time_position;

typedef struct motor_settings{
  float kP;
  float kI;
  float kD;
  float temperature_limit;
  float current_limit;
} motor_settings;

class can_servo{
  const uint8_t CAN_ID;
  uint8_t motor_status;
  uint8_t motor_mode;
  float movement;
  uint8_t last_status;
  uint8_t last_mode;
  float last_current;
  float last_temperature;

  public:
    uint8_t get_ID();
    void set_mode(uint8_t new_mode);
    void set_status(uint8_t new_status);
    uint8_t get_mode();
    uint8_t get_status();
    float get_last_current();
    float get_last_temperature();
  
    float get_angle_velocity();
    float set_angle_velocity(float new_angle_velocity);

    void set_PID(float P, float I, float D);
    void set_max_current(float max_current);
    void set_max_temperature(float max_temperature);
};

uint8_t can_servo::get_ID(){
  return CAN_ID;
}

void can_servo::set_mode(uint8_t new_mode){
  //call can
}

void can_servo::set_status(uint8_t new_status){
  motor_status = new_status;
  //call can bus
}

uint8_t can_servo::get_mode(){
  //call can bus
  return motor_mode;
}

uint8_t can_servo::get_status(){
  //call can bus
  return motor_status;
}

float can_servo::get_last_current(){
  //call can bus
  return last_current;
}

float can_servo::get_last_temperature(){
  //call can bus
  return last_temperature;
}


float can_servo::get_angle_velocity(){
  float current_angle = 0; //call can
  return current_angle;
}

float can_servo::set_angle_velocity(float new_angle_velocity){
  //call can
}



class leg{
  private:
    can_servo *hip;
    can_servo *knee;
    can_servo *ankle;
  
    uint8_t leg_name = '';

  public:
    leg(can_servo *hip, can_servo *knee, can_servo *ankle, uint8_t leg_name);
    void set_position(position target_position);
    void set_motor_status(uint8_t status);
    void set_motor_mode(uint8_t mode);
    void set_motor_pid(float p, float i, float d);
    float get_max_current();
    float get_max_temperature();
    void set_max_current(float max_current);
    void set_max_temperature(float max_temperature);
}

leg::leg(can_servo *hip, can_servo *knee, can_servo *ankle, uint8_t leg_name){
  this->hip = hip;
  this->knee = knee;
  this->ankle = ankle;
  this->leg_name = leg_name;
}

void leg::set_position(position target_position){
  // do IK here
}

void leg::set_motor_status(uint8_t status);{
  hip->set_status(status);
  knee->set_status(status);
  ankle->set_status(status);
}

void leg::set_motor_status(uint8_t mode);{
  hip->set_motor_mode(mode);
  knee->set_motor_mode(mode);
  ankle->set_motor_mode(mode);
}

void leg::set_motor_pid(float p, float i, float d){
  hip->set_PID(P, I, D);
  knee->set_PID(P, I, D);
  ankle->set_PID(P, I, D);
}

float leg::get_max_current(){
  float max_current = hip->get_last_current();
  
  float b = knee->get_last_current();

  if(b > max_current){
    max_current = b;
  }
  
  b = ankle->get_last_current();

  if(b > max_current){
    max_current = b;
  }
  return max_current;
}

float leg::get_max_current(){
  float max_temperature = hip->get_last_temperature();
  
  float b = knee->get_last_temperature();

  if(b > max_temperature){
    max_temperature = b;
  }
  
  b = ankle->get_last_temperature();

  if(b > max_temperature){
    max_temperature = b;
  }
  return max_temperature;
}

void leg::set_max_current(float max_current){
  hip->set_max_current(max_current);
  knee->set_max_current(max_current);
  ankle->set_max_current(max_current);
}
void leg::set_max_temperature(float max_temperature){
  hip->set_max_temperature(max_current);
  knee->set_max_temperature(max_current);
  ankle->set_max_temperature(max_current);
}

class robot{
  private:
    leg *leg_a;
    leg *leg_b;
    leg *leg_c;
    leg *leg_d;

    motor_settings *current_settings;
  public:
    void set_position_all(position target_position);
    void set_position_a(position target_position);
    void set_position_b(position target_position);
    void set_position_c(position target_position);
    void set_position_d(position target_position);
    void set_limits();
    void set_PID();
    void set_motor_mode();
};


