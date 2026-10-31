find_package(livox_ros_driver2 REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)

add_library(${PROJECT_NAME}.pre
        cloud_convert/cloud_convert.cc
        )

target_link_libraries(${PROJECT_NAME}.pre
        ${third_party_libs}
        )

# 使用 ament 的依赖管理
ament_target_dependencies(${PROJECT_NAME}.pre
    livox_ros_driver2
    pcl_conversions
    pcl_ros
)

target_include_directories(${PROJECT_NAME}.pre
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../..>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../../include>
        ${PCL_INCLUDE_DIRS}
    PRIVATE
        ${rosbag2_cpp_INCLUDE_DIRS}
        ${rosbag2_storage_INCLUDE_DIRS}
)

target_compile_definitions(${PROJECT_NAME}.pre PUBLIC ${PCL_DEFINITIONS})
target_link_libraries(${PROJECT_NAME}.pre ${PCL_LIBRARIES})