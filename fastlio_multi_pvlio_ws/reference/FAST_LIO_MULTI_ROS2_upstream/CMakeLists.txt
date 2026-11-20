cmake_minimum_required(VERSION 3.5)
project(fast_lio_multi)

# C++ version
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release)

add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\") 

# OpenMP
find_package(OpenMP REQUIRED)

# ROS2 packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(livox_ros_driver2 REQUIRED)

# PCL (ROS2)
find_package(PCL REQUIRED COMPONENTS common io filters kdtree)

# Eigen
find_package(Eigen3 REQUIRED)

include_directories(
    include
    ${EIGEN3_INCLUDE_DIR}
    ${PCL_INCLUDE_DIRS}
)

add_definitions(${PCL_DEFINITIONS})
link_directories(${PCL_LIBRARY_DIRS})

# ==========================================
# Library: preprocess + ikd_Tree
# ==========================================
add_library(fastlio_preprocess
    include/ikd-Tree/ikd_Tree.cpp
    src/preprocess.cpp
)

target_link_libraries(fastlio_preprocess
    ${PCL_LIBRARIES}
    OpenMP::OpenMP_CXX
)

ament_target_dependencies(fastlio_preprocess
    rclcpp
    sensor_msgs
    geometry_msgs
    pcl_conversions
    livox_ros_driver2
    tf2_eigen
)

# ==========================================
# Executable: ONLY laserMapping_adaptive
# ==========================================
set(COMMON_DEPENDENCIES
    rclcpp
    sensor_msgs
    nav_msgs
    geometry_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
    tf2_eigen
    pcl_conversions
    livox_ros_driver2
)

foreach(node IN ITEMS laserMapping_adaptive laserMapping_bundle laserMapping_async)
    add_executable(${node}
        src/${node}.cpp
    )
    target_link_libraries(${node}
        fastlio_preprocess
        ${PCL_LIBRARIES}
        OpenMP::OpenMP_CXX
    )
    ament_target_dependencies(${node}
        ${COMMON_DEPENDENCIES}
    )
endforeach()

# Install binaries & resources
install(TARGETS
    laserMapping_adaptive
    laserMapping_bundle
    laserMapping_async
    fastlio_preprocess
    DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
