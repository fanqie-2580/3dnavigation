find_package(OpenCV REQUIRED)

add_library(${PROJECT_NAME}.algo
        static_imu_init.cc
)

target_include_directories(${PROJECT_NAME}.algo
    PUBLIC
        ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}.algo
        glog
        gflags
        ${PROJECT_NAME}.common
        ${OpenCV_LIBS}
)