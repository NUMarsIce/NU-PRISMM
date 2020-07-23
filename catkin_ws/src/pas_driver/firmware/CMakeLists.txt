cmake_minimum_required(VERSION 2.8.3)

add_compile_options(-std=c++11)

include_directories(${ROS_LIB_DIR} ${CATKIN_PACKAGE_INCLUDE_DESTINATION} $ENV{ROS_WORKSPACE}src/arduino_libs/libraries/headers)

generate_arduino_firmware(pas_driver
  SRCS
    pas_driver.cpp
    ${ROS_LIB_DIR}/time.cpp
    $ENV{ROS_WORKSPACE}src/arduino_libs/libraries/MovingAverageFilter.cpp
    $ENV{ROS_WORKSPACE}src/arduino_libs/libraries/AccelStepper.cpp
    $ENV{ROS_WORKSPACE}src/arduino_libs/libraries/Thermistor.cpp
    $ENV{ROS_WORKSPACE}src/arduino_libs/libraries/ACS712_AC.cpp
    $ENV{ROS_WORKSPACE}src/arduino_libs/libraries/ACS712.cpp
    $ENV{ROS_WORKSPACE}src/arduino_libs/libraries/HX711.cpp
  BOARD mega2560
  PORT /dev/ttyACM1
)
