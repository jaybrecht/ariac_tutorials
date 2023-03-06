cmake_minimum_required(VERSION 3.8)
project(ariac_tutorials)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(ariac_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

# Install Python modules
ament_python_install_package(${PROJECT_NAME} SCRIPTS_DESTINATION lib/${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  nodes/start_competition.py
  nodes/read_sensor.py
  nodes/read_orders.py
  nodes/change_gripper_state.py
  nodes/move_robots.py
  nodes/move_agv.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install the config directory to the package share directory
install(DIRECTORY 
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install C++ executables
add_executable(robot_commander 
  src/robot_commander.cpp)

ament_target_dependencies(robot_commander 
  rclcpp
  moveit_ros_planning_interface 
  ariac_msgs)

install(TARGETS
  robot_commander
  DESTINATION lib/${PROJECT_NAME})

ament_package()
