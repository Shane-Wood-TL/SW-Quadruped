#include "../../include/motion_system/kinematics.h"
#include "../../include/math_functions.h"

kinematics::kinematics(leg *legV)
{
    legC = legV;
}

void kinematics::mainKinematics(position current_position)
{
    char leg_name =legC->getLegName();


    float adjusted_half_body_length = BODY_LENGTH/2.0f * cos(abs(degrees_to_radians(current_position.pitch)));

    float pitch_height_adjustment = 0;
    if (adjusted_half_body_length != BODY_LENGTH/2.0) {
        pitch_height_adjustment = pythagorean_side(adjusted_half_body_length, BODY_LENGTH/2.0);
    }


     if (leg_name == 'A' ||  leg_name == 'B') {
            if (current_position.pitch > 0.0) {
                current_position.z += pitch_height_adjustment;
            } else {
                current_position.z -= pitch_height_adjustment;
            }
        }else {
            if (current_position.pitch > 0.0) {
                current_position.z -= pitch_height_adjustment;
            } else {
                current_position.z += pitch_height_adjustment;
            }
        }
        
        if (current_position.pitch > 0.0) {
            current_position.y -= BODY_LENGTH/2.0 - adjusted_half_body_length;
        }else{
            current_position.y += BODY_LENGTH/2.0 - adjusted_half_body_length;
        }

        //roll
        float adjusted_half_body_width = (BODY_WIDTH/2.0)*cos(abs(degrees_to_radians(current_position.roll)));
        
        //fn pythagorean_side(a: f32, c: f32) -> f32 {
        float roll_height_adjustment = pythagorean_side(adjusted_half_body_width, BODY_WIDTH/2.0);
        
        
        if (leg_name == 'A' ||  leg_name == 'C') {
            if (current_position.roll > 0.0) {
            	current_position.x -= BODY_WIDTH/2.0 - adjusted_half_body_width;
            	current_position.z += roll_height_adjustment;
            }else{
            	current_position.x+= BODY_WIDTH/2.0 - adjusted_half_body_width;
            	current_position.z -= roll_height_adjustment;
            }
        } else {
            if (current_position.roll > 0.0) {
            	current_position.x+= BODY_WIDTH/2.0 - adjusted_half_body_width;
            	current_position.z -= roll_height_adjustment;
            }else{
            	current_position.x-= BODY_WIDTH/2.0 - adjusted_half_body_width;
            	current_position.z += roll_height_adjustment;
            }
        }
        
        
        
        //yaw
        float x_yaw_adjust = tan(abs(degrees_to_radians(current_position.yaw)))*(BODY_LENGTH/2.0);
        float y_yaw_adjust = tan(abs(degrees_to_radians(current_position.yaw)))*(BODY_WIDTH/2.0);
        
        if (current_position.yaw < 0.0) {
        	x_yaw_adjust = -x_yaw_adjust;
        	y_yaw_adjust = -y_yaw_adjust;
        }
        
        

        switch (leg_name) {
            case 'A':
                current_position.x -= x_yaw_adjust;
                current_position.y += y_yaw_adjust;
                break;
            case 'B':
                current_position.x += x_yaw_adjust;
                current_position.y -= y_yaw_adjust;
                break;
            case 'C':
                current_position.x -= x_yaw_adjust;
                current_position.y += y_yaw_adjust;
                break;
            case 'D':
                current_position.x += x_yaw_adjust;
                current_position.y += y_yaw_adjust;
                break;
            default:
                break;
        }
        
        
        float hip_angle;
        float knee_angle;
        float ankle_angle;
        
        //positional calculations

        //hip
        float inner_leg_length = pythagorean_hypotenuse(H_LENGTH_V + current_position.x, current_position.z);
        //println!("inner_leg_length: {}", inner_leg_length);
        
        //fn pythagorean_side(a: f32, c: f32) -> f32 {
        float outer_leg_length = pythagorean_side(H_LENGTH_V, inner_leg_length);
        //println!("outer_leg_length: {}", outer_leg_length);
        
        
        float hip_angle_a = atan((H_LENGTH_V + current_position.x) / current_position.z);
        //println!("hip_angle_a: {}", radians_to_degrees(hip_angle_a));
        float hip_angle_b = atan(outer_leg_length / H_LENGTH_V);
        //println!("hip_angle_b: {}", radians_to_degrees(hip_angle_b));
        
        hip_angle = radians_to_degrees(hip_angle_a + hip_angle_b);
        

        //knee and ankle
        float outer_leg_length_side = pythagorean_hypotenuse(abs(current_position.y), outer_leg_length);

        if (current_position.y == 0.0) {
            knee_angle = 90.0 + radians_to_degrees(law_of_cosines(A_LENGTH_V,outer_leg_length_side, B_LENGTH_V));
        } else if (current_position.y > 0.0) {
            float knee_angle_a = atan(current_position.y / outer_leg_length);
            float knee_angle_b = law_of_cosines(A_LENGTH_V, outer_leg_length_side, B_LENGTH_V);
            knee_angle = 90.0 + radians_to_degrees(knee_angle_b - knee_angle_a);
        } else {
            float knee_angle_a = atan(abs(current_position.y) / outer_leg_length);
            float knee_angle_b = law_of_cosines(A_LENGTH_V, outer_leg_length_side, B_LENGTH_V);
            knee_angle = 90.0 + radians_to_degrees(knee_angle_a + knee_angle_b);
        }

        ankle_angle = radians_to_degrees(law_of_cosines(A_LENGTH_V, B_LENGTH_V, outer_leg_length_side));

        legC->setAngles(hip_angle, knee_angle, ankle_angle);
}
