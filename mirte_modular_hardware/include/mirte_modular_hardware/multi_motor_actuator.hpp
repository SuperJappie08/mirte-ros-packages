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

#ifndef MIRTE_MODULAR_HARDWARE__MULTI_MOTOR_ACTUATOR_HPP_
#define MIRTE_MODULAR_HARDWARE__MULTI_MOTOR_ACTUATOR_HPP_

#include <string>
#include <vector>
/* FIXME(SuperJappie08): TO SEPERATE INCLUDES */
#include <hardware_interface/actuator_interface.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <realtime_tools/realtime_publisher.hpp>
/* FIXME(SuperJappie08): TO SEPERATE INCLUDES */
#include <rclcpp/macros.hpp>
#include <rclcpp/time.hpp>

#include "mirte_modular_hardware/visibility_control.hpp"
#include "mirte_msgs/msg/set_speed_named.hpp"
#include "mirte_msgs/msg/set_speed_named_array.hpp"

namespace mirte_modular_hardware
{

class MultiMotorActuator : public hardware_interface::ActuatorInterface
{
private:
  using MultiSpeedMsg = mirte_msgs::msg::SetSpeedNamedArray;
  using NamedSpeed = mirte_msgs::msg::SetSpeedNamed;

public:
  RCLCPP_SHARED_PTR_DEFINITIONS(MultiMotorActuator)

  MIRTE_MODULAR_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  MIRTE_MODULAR_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  MIRTE_MODULAR_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

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
  std::string topic_name_;

  struct MotorHandle
  {
    double max_speed;
    std::string interface_name;
    std::string tmx_motor_name;
    int idx = -1;

    int calculate_motor_value(double velocity) const { return (int)(velocity / max_speed * 100.0); }
  };
  std::vector<MotorHandle> motor_handles_;

  rclcpp::Node::SharedPtr node_ = nullptr;
  rclcpp::Node::SharedPtr get_node() const { return node_; }

  rclcpp::Publisher<MultiSpeedMsg>::SharedPtr multi_speed_publisher_ = nullptr;
  realtime_tools::RealtimePublisher<MultiSpeedMsg>::SharedPtr multi_speed_publisher_rt_ = nullptr;

  /// Retrieve the optional telemetrix motor name overide from the joint parameters.
  const std::string & retrieve_tmx_motor_name(
    const hardware_interface::ComponentInfo & joint) const;
};
}  // namespace mirte_modular_hardware

#endif  // MIRTE_MODULAR_HARDWARE__MULTI_MOTOR_ACTUATOR_HPP_
