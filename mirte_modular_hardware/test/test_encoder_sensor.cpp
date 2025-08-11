// Copyright 2025, TU Delft
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Authors: Jasper van Brakel

#include <gtest/gtest.h>

#include <cmath>
#include <string>
#if __has_include(<hardware_interface/hardware_interface/version.h>)
#include <hardware_interface/hardware_interface/version.h>
#else
#include <hardware_interface/version.h>
#endif
#include <hardware_interface/resource_manager.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/utilities.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <ros2_control_test_assets/descriptions.hpp>

TEST(TestEncoderSensor, load_sensor_encoder_single_encoder)
{
  const std::string sensor_encoder_single_encoder =
    R"(
  <ros2_control name="EncoderSensorSingleEncoder" type="sensor">
    <hardware>
      <plugin>mirte_modular_hardware/EncoderSensor</plugin>
      <param name="topic">some/encoder/topic</param>
      <param name="ticks_per_rotation">100</param>
      <param name="initial_message_timeout_ms">500</param>
    </hardware>
    <joint name="joint1">
      <state_interface name="position"/>
      <state_interface name="velocity"/>
    </joint>
  </ros2_control>
)";
  auto urdf = ros2_control_test_assets::urdf_head_continuous_missing_limits +
              sensor_encoder_single_encoder + ros2_control_test_assets::urdf_tail;
  auto node = std::make_shared<rclcpp::Node>("test_sensor_encoder_single_encoder");

// The API of the ResourceManager has changed in hardware_interface 5.3.0
#if HARDWARE_INTERFACE_VERSION_GTE(5, 3, 0)
  hardware_interface::ResourceManagerParams params;
  params.robot_description = urdf;
  params.clock = node->get_clock();
  params.logger = node->get_logger();
  params.executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  ASSERT_NO_THROW(hardware_interface::ResourceManager rm(params, true);
                  ASSERT_TRUE(rm.are_components_initialized()););
// The API of the ResourceManager has changed in hardware_interface 4.13.0
#elif HARDWARE_INTERFACE_VERSION_GTE(4, 13, 0)
  ASSERT_NO_THROW(
    hardware_interface::ResourceManager rm(
      urdf, node->get_node_clock_interface(), node->get_node_logging_interface(), false);
    ASSERT_TRUE(rm.are_components_initialized()););
#else
  ASSERT_NO_THROW(hardware_interface::ResourceManager rm(urdf, true, false));
#endif
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);

  return RUN_ALL_TESTS();
}
