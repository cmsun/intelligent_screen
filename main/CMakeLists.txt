set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

idf_component_register(SRCS "motion.cpp" "micro_ros_node.cpp" "mpu9250.cpp" "main.cpp"
                    INCLUDE_DIRS ".")
