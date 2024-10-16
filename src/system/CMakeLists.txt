cmake_minimum_required(VERSION 3.8)
project(system)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(format_data REQUIRED)


# Add the launch file installation section
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/
)

# Create executables for sensor and hub nodes
add_executable(sensor src/sensor.cpp)
ament_target_dependencies(sensor rclcpp std_msgs format_data)

add_executable(hub src/hub.cpp)
ament_target_dependencies(hub rclcpp std_msgs format_data)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Skip the linter which checks for copyrights
  set(ament_cmake_copyright_FOUND TRUE)
  # Skip cpplint (only works in a git repo)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Install sensor and hub executables
install(TARGETS
  sensor
  hub
  DESTINATION lib/${PROJECT_NAME})

# Add this line if you are using ROS 2 launch
ament_package()
