cmake_minimum_required(VERSION 3.8)
project(uav_agent)

# find dependencies
find_package(ament_cmake REQUIRED)

# Install Python executables
install(PROGRAMS
  uav_agent/uav_agent.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
