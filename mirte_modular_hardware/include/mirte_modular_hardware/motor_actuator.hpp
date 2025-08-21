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

#ifndef MIRTE_MODULAR_HARDWARE__MOTOR_ACTUATOR_HPP_
#define MIRTE_MODULAR_HARDWARE__MOTOR_ACTUATOR_HPP_

#include <limits>
/* FIXME(SuperJappie08): TO SEPERATE INCLUDES */
#include <hardware_interface/actuator_interface.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <realtime_tools/realtime_publisher.hpp>
/* FIXME(SuperJappie08): TO SEPERATE INCLUDES */
#include <rclcpp/macros.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp_lifecycle/state.hpp>
/* FIXME(SuperJappie08): TO SEPERATE INCLUDES */

#include "mirte_modular_hardware/hardware_interface_helper.hpp"
#include "mirte_modular_hardware/visibility_control.hpp"
#include "std_msgs/msg/int32.hpp"

#if !HARDWARE_INTERFACE_NODE_AVAILABLE
#include <rclcpp/node.hpp>
#endif

namespace mirte_modular_hardware
{

class MotorActuator : public hardware_interface::ActuatorInterface
{
private:
  using SpeedMsg = std_msgs::msg::Int32;

public:
  RCLCPP_SHARED_PTR_DEFINITIONS(MotorActuator)

  MIRTE_MODULAR_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_init(const HARDWARE_INTERFACE_INIT_PARAM & params) override;

  MIRTE_MODULAR_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  MIRTE_MODULAR_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  MIRTE_MODULAR_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  MIRTE_MODULAR_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  MIRTE_MODULAR_HARDWARE_PUBLIC
  hardware_interface::return_type perform_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces) override;

  MIRTE_MODULAR_HARDWARE_PUBLIC
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  MIRTE_MODULAR_HARDWARE_PUBLIC
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  double max_motor_speed_ = std::numeric_limits<double>::quiet_NaN();

#if !HARDWARE_INTERFACE_NODE_AVAILABLE
  rclcpp::Node::SharedPtr node_ = nullptr;
  rclcpp::Node::SharedPtr get_node() const { return node_; }
#endif

  rclcpp::Publisher<SpeedMsg>::SharedPtr speed_publisher_ = nullptr;
  realtime_tools::RealtimePublisher<SpeedMsg>::SharedPtr speed_publisher_rt_ = nullptr;
};

}  // namespace mirte_modular_hardware

#endif  // MIRTE_MODULAR_HARDWARE__MOTOR_ACTUATOR_HPP_
