cmake_minimum_required(VERSION 3.8)
project(tee_probe)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# Install directories (launch, config, urdf, scripts, models)
install(DIRECTORY
  launch
  config
  urdf
  scripts
  models
  DESTINATION share/${PROJECT_NAME}/
)

# Install Python scripts as executables
install(PROGRAMS
  scripts/drl_controller_node.py
  scripts/tee_probe_env.py
  scripts/train_tee_probe.py
  scripts/manual_control_node.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
