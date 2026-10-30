cmake_minimum_required(VERSION 3.8)
project(traj_utils)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE "Release")

add_compile_options(-Wall -O3 -g)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

rosidl_generate_interfaces(traj_utils
  "msg/Bspline.msg"
  "msg/DataDisp.msg"
  "msg/MultiBsplines.msg"
  "msg/TrajectoryExecutionState.msg"

  DEPENDENCIES std_msgs geometry_msgs builtin_interfaces
)

ament_export_dependencies(
  rosidl_default_runtime
  rclcpp
  std_msgs
  geometry_msgs
  visualization_msgs
  Eigen3
  PCL
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

include_directories( 
    SYSTEM 
    include 
    ${Eigen3_INCLUDE_DIRS} 
    ${PCL_INCLUDE_DIRS}
)

ament_export_include_directories(include)

link_directories(${PCL_LIBRARY_DIRS})

add_library( traj_utils_lib SHARED
    src/planning_visualization.cpp 
    src/polynomial_traj.cpp 
    )

target_include_directories(traj_utils_lib
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(traj_utils_lib std_msgs geometry_msgs visualization_msgs rclcpp Eigen3 PCL)

install(TARGETS traj_utils_lib
  EXPORT export_traj_utils_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

ament_export_libraries(traj_utils_lib)
ament_export_targets(export_traj_utils_lib HAS_LIBRARY_TARGET)

ament_package()
