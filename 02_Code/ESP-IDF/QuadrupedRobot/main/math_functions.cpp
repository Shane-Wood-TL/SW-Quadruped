
#include "../include/math_functions.h"
#include "../include/all_includes.h"

float radians_to_degrees(float radians){
  return radians * (180.0f / PI);
}

float degrees_to_radians(float degrees){
  return degrees * (PI / 180.0f);
}

float pythagorean_hypotenuse(float a, float b) {
    return std::sqrt(std::max(0.0f, a * a + b * b));
}

float pythagorean_side(float a, float c){
    return std::sqrt(std::max(0.0f, c * c - a * a));
}

float law_of_cosines(float a, float b, float c){
    float angle_c = ((a * a + b * b) - (c * c)) / (2.0f * a * b);
    angle_c = std::acos(angle_c);
    return angle_c;
}
