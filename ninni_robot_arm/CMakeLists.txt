cmake_minimum_required(VERSION 3.8)
project(ninni_robot_arm)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(arm_rotator src/arm_rotator.cpp)
ament_target_dependencies(arm_rotator rclcpp sensor_msgs)

install(TARGETS arm_rotator
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY urdf launch config rviz
  DESTINATION share/${PROJECT_NAME})

ament_package()
