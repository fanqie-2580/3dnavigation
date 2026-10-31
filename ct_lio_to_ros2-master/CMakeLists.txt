cmake_minimum_required(VERSION 3.8)
project(ct_lio)

MESSAGE(STATUS "[Release Mode]")
set(CMAKE_BUILD_TYPE "Release")

# 设置C++标准为C++17（推荐）
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# 设置编译选项
set(CMAKE_CXX_FLAGS "-O3 -pthread")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fexceptions")

# ROS 2 包
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(Ceres REQUIRED)
find_package(livox_ros_driver2 REQUIRED)

# 包含目录设置
add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")
include_directories(
  include 
  src
  thirdparty
  thirdparty/sophus
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)

# 添加子目录
add_subdirectory(src)

# 导出依赖项，使得下游包可以使用这些依赖
ament_export_dependencies(
  geometry_msgs
  nav_msgs
  sensor_msgs
  rclcpp
  std_msgs
  std_srvs
  rosbag2_cpp
  rosbag2_storage
  tf2
  tf2_ros
  livox_ros_driver2
)

install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
  FILES_MATCHING PATTERN "*.yaml"
)

install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
  FILES_MATCHING PATTERN "*.launch.py"
)

# 导出包含目录
ament_export_include_directories(
  include
  src
  thirdparty
  thirdparty/sophus
  livox_ros_driver2
)

ament_package()
