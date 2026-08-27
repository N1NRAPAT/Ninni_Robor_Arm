cmake_minimum_required(VERSION 3.8)
project(ninni_robot_arm)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# Python nodes
install(PROGRAMS
  src/camera_src/camera_pub.py
  src/camera_src/cuda_cv_node.py
  src/camera_src/stm32_serial_node.py
  src/servo_src/servo_bridge_node.py
  src/servo_src/servo_dance_test.py
  src/servo_src/servo_plotter_node.py
  src/movement_sim/pose_sequence_node.py
  src/movement_sim/ik_workspace_sweep.py
  src/movement_sim/ik_analytic_node.py
  src/integrated_src/yolo_detect_node.py
  src/integrated_src/gesture_control_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY urdf launch config rviz meshes
  DESTINATION share/${PROJECT_NAME})

ament_package()
