# 查找 livox_ros_driver2 依赖
find_package(livox_ros_driver2 REQUIRED)

add_library(${PROJECT_NAME}.tools INTERFACE)

# 设置包含目录
target_include_directories(${PROJECT_NAME}.tools
    INTERFACE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/..>
        $<INSTALL_INTERFACE:include/${PROJECT_NAME}/tools>
)