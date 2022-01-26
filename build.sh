#!/usr/bin/sh

# Cheap Method.

source /opt/ros/foxy/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=1 --packages-select cobalt_utils --allow-overriding cobalt_utils
source ./install/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=1 --packages-select x3_sim --allow-overriding x3_sim
source ./install/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=1 --packages-select husky_sim --allow-overriding husky_sim
source ./install/setup.bash