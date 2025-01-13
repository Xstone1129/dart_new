cmake_minimum_required(VERSION 3.10)
project(light_detector)

## Use C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
## enforcing cleaner code.
#add_definitions(-Wall -Werror)

## Export compile commands for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

## 禁用不必要的警告：未使用的变量和符号比较警告
#add_compile_options(-Wno-unused-variable)
#add_compile_options(-Wno-sign-compare)

#######################
## Find dependencies ##
#######################


find_package(ament_cmake_auto REQUIRED)
find_package(OpenCV REQUIRED)
ament_auto_find_build_dependencies()

###########
## Build ##
###########

ament_auto_add_library(${PROJECT_NAME} SHARED
  DIRECTORY src
)

target_include_directories(${PROJECT_NAME} PUBLIC ${OpenCV_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})

# # Add the executable
# ament_add_executable(exchanger_detector
#   src/detector_node.cpp
# )

# Register the component
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN rm_auto_light::DetectorNode
  EXECUTABLE detect_node
)

#############
## Testing ##
#############

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   list(APPEND AMENT_LINT_AUTO_EXCLUDE
#     ament_cmake_copyright
#     ament_cmake_uncrustify
#     ament_cmake_cpplint
#   )
#   ament_lint_auto_find_test_dependencies()
#endif()

#############
## Install ##
#############

ament_auto_package(
  INSTALL_TO_SHARE
)
