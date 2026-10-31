cmake_minimum_required(VERSION 3.8)
project(fastloc)

MESSAGE(STATUS "[Release Mode]")
set(CMAKE_BUILD_TYPE "Release")

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -pthread -fexceptions")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fexceptions")

# 处理 CMake 策略警告
if(POLICY CMP0074)
  cmake_policy(SET CMP0074 NEW)
endif()

# ROS 2 包
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rosbag2_cpp REQUIRED)

# 第三方库
include(cmake/packages.cmake)

# Eigen3, PCL, Ceres 等应在 packages.cmake 中找到
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(Ceres REQUIRED)

add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

# 包含目录
include_directories(
  include 
)

include_directories(${PROJECT_SOURCE_DIR})
include_directories(${PROJECT_SOURCE_DIR}/src)
include_directories(${PROJECT_SOURCE_DIR}/thirdparty/sophus)

# 包含第三方库目录
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${PCL_INCLUDE_DIRS})
include_directories(${CERES_INCLUDE_DIRS})

add_subdirectory(src)

# 导出依赖
ament_export_dependencies(
  geometry_msgs
  nav_msgs
  sensor_msgs
  rclcpp
  std_msgs
  tf2
  tf2_ros
  Eigen3
  rosbag2_cpp
)

ament_export_include_directories(include)

ament_package()