cmake_minimum_required(VERSION 3.8)
project(planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)

add_executable(pub_map src/test.cpp)
ament_target_dependencies(pub_map rclcpp nav_msgs)

install(TARGETS
  pub_map
  DESTINATION lib/${PROJECT_NAME})

ament_package()
