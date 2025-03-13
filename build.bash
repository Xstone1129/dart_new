#!/bin/bash

# 步骤 1: 构建工作空间
colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON && \

# colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug -DCMAKE_EXPORT_COMPILE_COMMANDS=ON --packages-select rm_exchanger_detector rm_vision_main && \

source install/setup.sh && \

ros2 launch rm_vision_main vision_bringup.launch.py