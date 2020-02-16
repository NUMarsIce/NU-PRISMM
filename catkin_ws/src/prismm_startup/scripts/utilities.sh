#!/bin/bash

_require_ros_workspace() {
    if [ -z "$ROS_WORKSPACE" ]; then
        echo "Must have ROS_WORKSPACE set to current workspace"
        return 1
    else
        return 0
    fi
}

flash_hammer() {
  catkin build --no-deps hammer_driver --make-args hammer_driver_firmware_hammer_driver-upload
}

flash_et() {
  catkin build --no-deps et_driver --make-args et_driver_firmware_et_driver-upload
}

flash_distil() {
  catkin build --no-deps distil_driver --make-args distil_driver_firmware_distil_driver-upload
}

obc_ros_build() {
  catkin build pudle_msgs jet_manager pudle_startup
}
