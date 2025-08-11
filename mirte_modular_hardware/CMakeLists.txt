cmake_minimum_required(VERSION 3.12)
project(mirte_modular_hardware)

# Default to C99 (ROS Jazzy Default)
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17 (ROS Jazzy Default)
# NOTE(SuperJappie08): Overrule C++ version to C++20 for numeric constants.
#                      Otherwise boost is required.
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/encoder_sensor.cpp
  src/helpers.cpp
  src/motor_actuator.cpp
  src/multi_motor_actuator.cpp
)

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface mirte_modular_hardware.xml)

if(BUILD_TESTING)
  ament_auto_find_test_dependencies()
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  # GTests
  ament_auto_add_gtest(test_encoder_sensor test/test_encoder_sensor.cpp)

  ament_auto_add_gtest(test_motor_actuator test/test_motor_actuator.cpp)

  ament_auto_add_gtest(test_multi_motor_actuator test/test_multi_motor_actuator.cpp)
endif()

ament_auto_package(
  USE_SCOPED_HEADER_INSTALL_DIR  # Pre-Kilted Fix
  INSTALL_TO_SHARE
    urdf
)
