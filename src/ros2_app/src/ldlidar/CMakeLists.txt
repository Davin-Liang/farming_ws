cmake_minimum_required(VERSION 3.5)
project(ldlidar)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Bin and Install
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../ldlidar_driver/include
)

file(GLOB  DRIVER_SRC ${CMAKE_CURRENT_SOURCE_DIR}/../../../ldlidar_driver/src/*.cpp)

add_executable(stp23_ros2node  ${CMAKE_CURRENT_SOURCE_DIR}/src/ros2_node/stp23_ros2node.cpp ${DRIVER_SRC})
ament_target_dependencies(stp23_ros2node rclcpp sensor_msgs)
target_link_libraries(stp23_ros2node pthread)

add_executable(stp23l_ros2node  ${CMAKE_CURRENT_SOURCE_DIR}/src/ros2_node/stp23l_ros2node.cpp ${DRIVER_SRC})
ament_target_dependencies(stp23l_ros2node rclcpp sensor_msgs)
target_link_libraries(stp23l_ros2node pthread)

install(TARGETS  stp23_ros2node stp23l_ros2node
 DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch rviz2
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()

