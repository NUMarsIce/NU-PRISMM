cmake_minimum_required(VERSION 2.8.3)
project(prismm_msgs)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
)

# Generate messages in the 'msg' folder
 add_message_files(
   FILES
   pas_data.msg
   dam_data.msg
 )

# Generate services in the 'srv' folder
 add_service_files(
   FILES
   getBool.srv
 )


# Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs  # Or other packages containing msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES prismm_msgs
  CATKIN_DEPENDS message_generation std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

