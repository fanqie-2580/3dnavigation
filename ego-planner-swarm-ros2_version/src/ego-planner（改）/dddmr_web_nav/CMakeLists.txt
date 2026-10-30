cmake_minimum_required(VERSION 3.5)
project(dddmr_web_nav)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/web_map_downsampler.py
  scripts/web_status_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  web
  DESTINATION share/${PROJECT_NAME}
)

if(UNIX)
  execute_process(
    COMMAND chmod +x
    ${CMAKE_CURRENT_SOURCE_DIR}/scripts/web_map_downsampler.py
    ${CMAKE_CURRENT_SOURCE_DIR}/scripts/web_status_node.py
    RESULT_VARIABLE DDDMR_WEB_NAV_CHMOD_RESULT
  )
  if(NOT DDDMR_WEB_NAV_CHMOD_RESULT EQUAL 0)
    message(WARNING "Failed to mark dddmr_web_nav helper scripts executable.")
  endif()
endif()

ament_package()
