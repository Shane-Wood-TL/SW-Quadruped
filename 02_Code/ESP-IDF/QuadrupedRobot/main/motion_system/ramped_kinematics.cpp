#include "../../include/motion_system/ramped_kinematics.h"

ramped_kinematics::ramped_kinematics(kinematics *kinematic_driver, rampLeg *ramped_driver){
    this->kinematic_driver = kinematic_driver;
    this->ramped_driver = ramped_driver;
}

void ramped_kinematics::update(){
    ramped_driver->update();
    single_leg current_postions = {ramped_driver->heightAt(),ramped_driver->fbAt(),ramped_driver->lrAt(),0,0,0};
    kinematic_driver->mainKinematics(current_postions);
}


void ramped_kinematics::set_stance(single_leg new_stance){
    ramped_driver->setPositions(new_stance.xH,new_stance.xLR,new_stance.xFB,0);
    update();
}

void ramped_kinematics::set_interpolated_stance(single_leg new_stance, float time_to_stance){
    ramped_driver->setPositions(new_stance.xH,new_stance.xLR,new_stance.xFB,time_to_stance);
    update();
}

void ramped_kinematics::stop(){
    ramped_driver->setPositions(ramped_driver->heightAt(), ramped_driver->lrAt(),ramped_driver->fbAt(),0);
    update();
}

bool ramped_kinematics::all_done(){
    return ramped_driver->allDone();
}