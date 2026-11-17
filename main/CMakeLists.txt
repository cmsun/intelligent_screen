set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

idf_component_register(SRCS "serial_comm.cpp" "setup_motor_id.cpp" "battery.cpp"
                            "bus_servo.cpp"
                            "motion.cpp"
                            "mpu9250.cpp"
                            "xl9535.cpp"
                            "ir_receiver.cpp"
                            "main.cpp"
                    INCLUDE_DIRS ".")
