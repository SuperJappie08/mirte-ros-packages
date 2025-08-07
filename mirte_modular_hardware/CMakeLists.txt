cmake_minimum_required(VERSION 3.8)
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
find_package(ament_cmake REQUIRED)
find_package(backward_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(mirte_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pluginlib REQUIRED)

include_directories(include)

add_library(${PROJECT_NAME} SHARED
  src/encoder_sensor.cpp
  src/helpers.cpp
  src/motor_actuator.cpp
  src/multi_motor_actuator.cpp
)

# # TODO: Is this desired???
# target_include_directories(${PROJECT_NAME} PUBLIC
#   $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
# )

# ament_target_dependencies will be deprecated
target_link_libraries(${PROJECT_NAME}
PUBLIC
  ${mirte_msgs_TARGETS}
  ${std_msgs_TARGETS}
  hardware_interface::hardware_interface
  pluginlib::pluginlib
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  realtime_tools::realtime_tools
)


# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface mirte_modular_hardware.xml)

# Install
install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(
  DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}/
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  backward_ros
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  realtime_tools
  pluginlib
  mirte_msgs
  std_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ros2_control_test_assets REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)

  # GTests
  set(THIS_PACKAGE_HW_INTERFACES_TEST_DEPENDS
    rclcpp::rclcpp
    hardware_interface::hardware_interface
    ros2_control_test_assets::ros2_control_test_assets
  )

  ament_add_gtest(test_encoder_sensor test/test_encoder_sensor.cpp)
  target_link_libraries(test_encoder_sensor ${PROJECT_NAME})
  target_link_libraries(test_encoder_sensor ${THIS_PACKAGE_HW_INTERFACES_TEST_DEPENDS})

  ament_add_gtest(test_motor_actuator test/test_motor_actuator.cpp)
  target_link_libraries(test_motor_actuator ${PROJECT_NAME})
  target_link_libraries(test_motor_actuator ${THIS_PACKAGE_HW_INTERFACES_TEST_DEPENDS})

  ament_add_gtest(test_multi_motor_actuator test/test_multi_motor_actuator.cpp)
  target_link_libraries(test_multi_motor_actuator ${PROJECT_NAME})
  target_link_libraries(test_multi_motor_actuator ${THIS_PACKAGE_HW_INTERFACES_TEST_DEPENDS})
endif()

ament_package()
