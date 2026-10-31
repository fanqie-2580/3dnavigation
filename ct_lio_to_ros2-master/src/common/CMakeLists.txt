add_library(${PROJECT_NAME}.common
        timer/timer.cc
        utility.cpp
        utils_trans.cpp
        utils.cpp
        global_flags.cc
        )

target_link_libraries(${PROJECT_NAME}.common
        ${third_party_libs}
        )

# 如果需要特定的包含目录，可以添加以下内容：
target_include_directories(${PROJECT_NAME}.common
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/..>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../../thirdparty>
)