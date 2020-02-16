cmake_minimum_required(VERSION 2.8.3)

add_compile_options(-std=c++11)

include_directories(${ROS_LIB_DIR} ${CATKIN_PACKAGE_INCLUDE_DESTINATION} $ENV{ROS_WORKSPACE}src/arduino_libs/libraries/headers)

# Remove this if using an Arduino without native USB (eg, other than Leonardo)
add_definitions(-DUSB_CON)

generate_arduino_firmware(dam_driver
  SRCS
    dam_driver.cpp
    ${ROS_LIB_DIR}/time.cpp
    $ENV{ROS_WORKSPACE}src/arduino_libs/libraries/AccelStepper.cpp
    $ENV{ROS_WORKSPACE}src/arduino_libs/libraries/DutyCycleCalc.cpp
    $ENV{ROS_WORKSPACE}src/arduino_libs/libraries/Thermistor.cpp
    $ENV{ROS_WORKSPACE}src/arduino_libs/libraries/ACS712_AC.cpp
    $ENV{ROS_WORKSPACE}src/arduino_libs/libraries/ACS712.cpp
  BOARD mega2560
  PORT /dev/ttyACM0
)
