cmake_minimum_required(VERSION 3.16)
project(joystick_cpp LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(spdlog REQUIRED)
find_package(Threads REQUIRED)

add_executable(joystick_teleop joystick_teleop.cpp)
target_include_directories(joystick_teleop PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(joystick_teleop PRIVATE spdlog::spdlog Threads::Threads)
