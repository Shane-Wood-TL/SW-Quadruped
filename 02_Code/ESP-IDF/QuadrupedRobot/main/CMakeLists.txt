idf_component_register(SRCS "math_functions.cpp" "./drivers/ACS71240.cpp" "./drivers/bno055.cpp" "./motion_system/kinematics.cpp" "./motion_system/ramped_kinematics.cpp"
                            "./motion_system/leg.cpp" "./motion_system/ramp_leg.cpp" "./interpolators/pid.cpp" "./interpolators/interpolation.cpp" "./motion_system/robot_movement.cpp"
                            "./drivers/pca9685.cpp" "./drivers/font.cpp" "./drivers/ssd1306.cpp" "main.cpp" "./motion_system/motor.cpp" "./motion_system/positioning/cycle.cpp"
                    INCLUDE_DIRS ".")