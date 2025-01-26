#include "../../include/motion_system/robot_movement.h"

robot_movement::robot_movement(ramped_kinematics *a_leg, ramped_kinematics *b_leg, ramped_kinematics *c_leg, ramped_kinematics *d_leg, cycle *starting_cycle){
    this->a_leg = a_leg;
    this->b_leg = b_leg;
    this->c_leg = c_leg;
    this->d_leg = d_leg;
    this->current_cycle = starting_cycle;
}

void robot_movement::update(){
    static stance current_stance;
    static float stance_time=0;
    if(all_done()){
        current_cycle->next_stance();
        current_stance = current_cycle->get_current_stance();
        stance_time = current_cycle->get_stance_time();
        printf("%f",current_stance.A_positions->xH);
        a_leg->set_interpolated_stance(*current_stance.A_positions,stance_time);
        b_leg->set_interpolated_stance(*current_stance.B_positions,stance_time);
        d_leg->set_interpolated_stance(*current_stance.C_positions,stance_time);
        d_leg->set_interpolated_stance(*current_stance.D_positions,stance_time);
    }
    a_leg->update();
    b_leg->update();
    c_leg->update();
    d_leg->update();
}
        
void robot_movement::set_cycle(cycle *current_cycle){
    this->current_cycle = current_cycle;
}




bool robot_movement::all_done(){
    return (a_leg->all_done() && b_leg->all_done() && c_leg->all_done() && d_leg->all_done());
}