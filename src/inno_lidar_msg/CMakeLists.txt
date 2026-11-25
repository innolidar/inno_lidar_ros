cmake_minimum_required(VERSION 3.5)

project(inno_lidar_msg)
# 明确设置CMP0057策略
if(POLICY CMP0057)
  cmake_policy(SET CMP0057 NEW)
endif()
#=======================================
# Compile setup (ORIGINAL, CATKIN, COLCON)
#=======================================
set(COMPILE_METHOD CATKIN)

if(COMPILE_METHOD STREQUAL "COLCON")
option(ENABLE_ROS1 "Enable ROS1" OFF)
option(ENABLE_ROS2 "Enable ROS2" ON)
else()
option(ENABLE_ROS1 "Enable ROS1" ON)
option(ENABLE_ROS2 "Enable ROS2" OFF)
endif()

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(${ENABLE_ROS1})
## Find dependencies
find_package(catkin REQUIRED COMPONENTS
  std_msgs
  message_generation
)

## Message files
add_message_files(
  FILES
  DeviceStatus.msg
)

## Generate message code
generate_messages(
  DEPENDENCIES
  std_msgs
)

## Declare catkin package
catkin_package(
  CATKIN_DEPENDS std_msgs message_runtime
)
endif(${ENABLE_ROS1})

if(${ENABLE_ROS2})
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DeviceStatus.msg"
  DEPENDENCIES std_msgs geometry_msgs
)
#ament_export_dependencies(rosidl_default_runtime)
#ament_export_include_directories(include)

ament_package()
endif(${ENABLE_ROS2})
