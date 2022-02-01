#!/usr/bin/sh

# Cheap Method.

ROS2 #alias for source /opt/ros/foxy/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=1 --packages-select cobalt_utils --allow-overriding cobalt_utils
srcit #alias for source ./install/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=1 --packages-select x3_sim --allow-overriding x3_sim
srcit
colcon build --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=1 --packages-select husky_sim --allow-overriding husky_sim
srcit
colcon build --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=1 --packages-select exp_r3_sim --allow-overriding exp_r3_sim
srcit