cmake_minimum_required(VERSION 3.8)
project(pv_lio)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -g -pthread")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fexceptions")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -fexceptions")

add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

message("Current CPU architecture: ${CMAKE_SYSTEM_PROCESSOR}")
if(CMAKE_SYSTEM_PROCESSOR MATCHES "(x86)|(X86)|(amd64)|(AMD64)")
  include(ProcessorCount)
  ProcessorCount(N)
  message("Processor number: ${N}")
  if(N GREATER 17)
    add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM=12)
    message("core for MP: 12")
  elseif(N GREATER 10)
    add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM=8)
    message("core for MP: 8")
  elseif(N GREATER 7)
    add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM=6)
    message("core for MP: 6")
  elseif(N GREATER 3)
    add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM=3)
    message("core for MP: 3")
  else()
    add_definitions(-DMP_PROC_NUM=1)
  endif()
else()
  add_definitions(-DMP_PROC_NUM=1)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(livox_ros_driver2 REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(PCL 1.8 REQUIRED)
find_package(Boost REQUIRED COMPONENTS timer)

find_package(OpenMP QUIET)
if(OpenMP_CXX_FOUND)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
endif()

message("Eigen: ${EIGEN3_INCLUDE_DIR}")

include_directories(
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  include
)

set(ROS2_DEPS
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  std_srvs
  visualization_msgs
  tf2
  tf2_ros
  tf2_eigen
  pcl_conversions
  livox_ros_driver2
)

add_executable(pv_lio_node src/voxelMapping.cpp src/preprocess.cpp)
target_link_libraries(pv_lio_node ${PCL_LIBRARIES})
ament_target_dependencies(pv_lio_node ${ROS2_DEPS})

add_executable(pandar_to_velodyne src/converter/pandar_to_velodyne.cpp)
target_link_libraries(pandar_to_velodyne ${PCL_LIBRARIES})
ament_target_dependencies(pandar_to_velodyne rclcpp sensor_msgs pcl_conversions)

add_executable(rs_to_velodyne src/converter/rs_to_velodyne.cpp)
target_link_libraries(rs_to_velodyne ${PCL_LIBRARIES})
ament_target_dependencies(rs_to_velodyne rclcpp sensor_msgs pcl_conversions)

install(TARGETS
  pv_lio_node
  pandar_to_velodyne
  rs_to_velodyne
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
