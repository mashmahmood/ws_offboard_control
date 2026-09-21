cmake_minimum_required(VERSION 3.5)
project(px4_ros_com)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(rclpy REQUIRED)

###########
# Python ##
###########

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  src/offboard_control/processes.py
  src/offboard_control/offboard_control_positional.py
  src/offboard_control/control_keyboard.py
  src/offboard_control/px4_odom_converter.py
  src/offboard_control/slam_to_px4.py
  src/services/slam_service.py
  DESTINATION lib/${PROJECT_NAME}
)

############
# Install ##
############

# Install launch files.
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)

# Install SLAM Toolbox / Nav2 parameter files.
install(DIRECTORY config DESTINATION share/${PROJECT_NAME}/)

ament_package()
