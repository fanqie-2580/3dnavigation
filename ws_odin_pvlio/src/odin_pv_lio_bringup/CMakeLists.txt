cmake_minimum_required(VERSION 3.8)
project(odin_pv_lio_bringup)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/odin_cloud_to_pvlio
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# optional empty rviz dir
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/rviz")
  install(DIRECTORY rviz DESTINATION share/${PROJECT_NAME})
endif()

ament_package()
