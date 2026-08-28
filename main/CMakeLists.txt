set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

idf_component_register(SRCS "esp32_serial_transport.c" "micro_ros.cpp" "motion.cpp" "mpu9250.cpp" "main.cpp"
                    INCLUDE_DIRS ".")
