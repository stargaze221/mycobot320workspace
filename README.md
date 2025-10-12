# mycobot320workspace

## Add the following to .bashrc
export ISAAC_ROS_WS=${HOME}/mycobot320workspace/isaac_ros-dev/
alias isaacrosdev='cd ${ISAAC_ROS_WS}/src/isaac_ros_common && ./scripts/run_dev.sh'

## This repository uses the following submodules:
- isaac_ros_common
- isaac_ros_nvblox
And it needs git submodule update --init --recursive

## I added Dockerfile.zed
Now, it added rosdep install given ros-img-dev.

## To run the ZED camera wrapper, use the below.
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2i

## To run the ZED NVblox example, use the below.
ros2 launch nvblox_examples_bringup zed_example.launch.py camera:=zed2