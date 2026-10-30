cmake_minimum_required(VERSION 3.8)
project(go2_cmd_vel_handler)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(unitree_api REQUIRED)

add_executable(go2_bridge_node 
    src/go2_cmd_vel_handler.cpp
    /home/ll/unitree_ros2/example/src/src/common/ros2_sport_client.cpp
)

# 关键修正：同时包含 include 目录和 include/common 目录
target_include_directories(go2_bridge_node SYSTEM PRIVATE
    /home/ll/unitree_ros2/example/src/include
    /home/ll/unitree_ros2/example/src/include/common
)

ament_target_dependencies(go2_bridge_node
    rclcpp
    geometry_msgs
    std_msgs
    unitree_api
)

install(TARGETS
    go2_bridge_node
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()