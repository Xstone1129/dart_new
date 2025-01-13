cmake_minimum_required(VERSION 3.10)
project(rm_vision_main)

## Use C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
## enforcing cleaner code.
add_definitions(-Wall -Werror)

## Export compile commands for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

#######################
## Find dependencies ##
#######################

# Find required packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_cmake_auto REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(geometry_msgs REQUIRED)

ament_auto_find_build_dependencies()  # Automatically find build dependencies

###########
## Build ##
###########

# Define your target executable or libraries
# Example: If you have executable source, add it like this:
# add_executable(rm_vision_main_node src/rm_vision_main_node.cpp)

# ament_target_dependencies(rm_vision_main_node rclcpp std_msgs sensor_msgs OpenCV geometry_msgs)

#############
## Install ##
#############

# Install directories (like launch and config)
install(DIRECTORY launch
  DESTINATION .)

# Automatically install the package share directories
ament_auto_package(
  INSTALL_TO_SHARE
  launch
  config
)
