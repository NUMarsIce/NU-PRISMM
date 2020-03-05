cmake_minimum_required(VERSION 2.8.3)
project(dam_driver)

find_package(catkin REQUIRED COMPONENTS
  rosserial_arduino
  rosserial_client
  std_msgs
  prismm_msgs
)

include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})

catkin_package(
    DEPENDS rosserial_arduino rosserial_client roscpp
    CATKIN_DEPENDS prismm_msgs
)

rosserial_generate_ros_lib(
  PACKAGE rosserial_arduino
  SCRIPT make_libraries.py
)

add_dependencies(dam_driver_ros_lib
  ${catkin_EXPORTED_TARGETS}
  prismm_msgs_generate_messages_cpp
)

rosserial_configure_client(
  DIRECTORY firmware
  TOOLCHAIN_FILE ${ROSSERIAL_ARDUINO_TOOLCHAIN}
)

rosserial_add_client_target(firmware dam_driver ALL)
rosserial_add_client_target(firmware dam_driver-upload)

