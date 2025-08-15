cmake_minimum_required(VERSION 3.8)
project(drone_agent)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Install Python executables
install(PROGRAMS
  scripts/drone_agent.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python modules (directories)
install(DIRECTORY
  scripts/tools
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python modules (individual files)
install(FILES
  scripts/help.py
  scripts/llm.py
  scripts/prompts.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
