# mycobot320workspace

## Add the following to .bashrc
export ISAAC_ROS_WS=${HOME}/mycobot320workspace/isaac_ros-dev/
alias isaacrosdev='cd ${ISAAC_ROS_WS}/src/isaac_ros_common && ./scripts/run_dev.sh'

## This repository uses the following submodules:
- isaac_ros_common
- isaac_ros_nvblox

## The following was necesaary for colcon build.
cd ${ISAAC_ROS_WS}/src/isaac_ros_nvblox
git submodule update --init --recursive