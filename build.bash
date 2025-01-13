#!/bin/bash

# 步骤 1: 构建工作空间
colcon build


source install/setup.bash

# 步骤 3: 启动 ROS 2 启动文件
ros2 launch rm_vision_main vision_bringup.launch.py