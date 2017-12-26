# the minimum version of CMake required
cmake_minimum_required(VERSION 2.8)

# we add the sub-directories that we want CMake to scan
add_subdirectory(libcreate EXCLUDE_FROM_ALL)

add_executable(joystick_control joystick_control.cpp)

# Specify libraries to link executable targets against
target_link_libraries(joystick_control
  create
)

set_property(TARGET joystick_control PROPERTY COMPILE_OPTIONS "-std=c++11")
