#!/bin/bash

echo '>>>>> Building Messages <<<<<<'
cd ~/nu-prismm/catkin_ws/
source devel/setup.bash
echo '>>>>> Building Ros Lib <<<<<<'
rm -r ~/Arduino/libraries/ros_lib
rosrun rosserial_arduino make_libraries.py ~/Arduino/libraries
rm -rf ~/Arduino/libraries/ros_lib ~/nu-prismm/catkin_ws/build/dam_driver/
cp -r ~/Arduino/libraries/ros_lib ~/nu-prismm/catkin_ws/build/dam_driver/
rm -rf ~/Arduino/libraries/ros_lib ~/nu-prismm/catkin_ws/build/pas_driver/
cp -r ~/Arduino/libraries/ros_lib ~/nu-prismm/catkin_ws/build/pas_driver/
echo '>>>>> Building NUPRISMM Code <<<<<'
source devel/setup.bash
catkin_make
