cmake_minimum_required(VERSION 3.8)
project(planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_subdirectory(../aStar ${CMAKE_BINARY_DIR}/aStar_build)

add_executable(test_planner_a_star src/node_a_star.cpp)
target_link_libraries(test_planner_a_star aStar utils)
ament_target_dependencies(test_planner_a_star rclcpp nav_msgs geometry_msgs)

add_executable(test_plan src/test.cpp)
target_link_libraries(test_plan aStar utils)
ament_target_dependencies(test_plan rclcpp nav_msgs geometry_msgs)

install(TARGETS
  test_plan
  test_planner_a_star
  DESTINATION lib/${PROJECT_NAME})

# Install launch files.
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
