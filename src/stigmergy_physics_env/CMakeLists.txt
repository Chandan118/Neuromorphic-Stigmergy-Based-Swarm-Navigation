cmake_minimum_required(VERSION 3.8)
project(stigmergy_physics_env)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(stigmergy_node src/stigmergy_node.cpp)
ament_target_dependencies(stigmergy_node rclcpp nav_msgs std_msgs)

install(TARGETS
  stigmergy_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
