idf_component_register(SRCS "./drivers/ACS71240.cpp" "./drivers/bno055.cpp" "./motion_system/kinematics.cpp"
                            "./motion_system/leg.cpp" "./motion_system/ramp_leg.cpp" "./interpolators/pid.cpp" "./interpolators/interpolation.cpp"
                            "./drivers/pca9685.cpp" "./drivers/font.cpp" "./drivers/ssd1306.cpp" "main.cpp" "./motion_system/motor.cpp"
                    INCLUDE_DIRS ".")