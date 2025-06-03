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

/* FIXME(SuperJappie08): TO SEPERATE INCLUDES */
#include <hardware_interface/actuator_interface.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/lexical_casts.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <hardware_interface/types/lifecycle_state_names.hpp>
#include <realtime_tools/realtime_publisher.hpp>
/* FIXME(SuperJappie08): TO SEPERATE INCLUDES */
#include <rclcpp/duration.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher_options.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include "mirte_modular_hardware/motor_actuator.hpp"
#include "std_msgs/msg/int32.hpp"

namespace mirte_modular_hardware
{

const std::string MOTOR_ACTUATOR_NODE_NAME_PREFIX = "mirte_modular_hardware_motor_actuator_";

hardware_interface::CallbackReturn MotorActuator::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::ActuatorInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Retrieve general parameters

  // FIXME(SuperJappie08): Add configurable speed scaling factor

  std::string topic_name;
  if (auto topic_pair = info_.hardware_parameters.find("topic");
      topic_pair != info_.hardware_parameters.end()) {
    RCLCPP_INFO(
      get_logger(), "Using '%s' as the topic name (relative to the hardware node).",
      topic_pair->second.c_str());
    topic_name = topic_pair->second;
  } else {
    // TODO(SuperJappie08): Consider making this parameter optional.
    RCLCPP_FATAL(
      get_logger(),
      "Missing the required 'topic' hardware parameter, to indicate the topic (relative to the "
      "the hardware node).");
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (auto max_motor_speed_raw = info_.hardware_parameters.find("max_motor_speed");
      max_motor_speed_raw != info_.hardware_parameters.end()) {
    max_motor_speed_ = hardware_interface::stod(max_motor_speed_raw->second);
    RCLCPP_INFO(get_logger(), "Loaded 'max_motor_speed' [%f rad/s]", max_motor_speed_);
  } else {
    // TODO(SuperJappie08): Consider making this parameter optional
    RCLCPP_FATAL(
      get_logger(),
      "Missing the required 'max_motor_speed' hardware parameter [double, rad/s]. This is used in "
      "the conversion to the percentage based command speed.");
    return hardware_interface::CallbackReturn::ERROR;
  }

  // TODO(SuperJappie08): Retrieve general params.

  // Optional hardware parameters
  // TODO(SuperJappie08): Retrieve optional params

  // Validate if the configuration is valid.

  if (!info_.transmissions.empty()) {
    RCLCPP_FATAL(
      get_logger(),
      "Transmission components are not supported on the '%s' interface type, but they were "
      "defined.",
      info_.hardware_plugin_name.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (!info_.sensors.empty()) {
    RCLCPP_FATAL(
      get_logger(),
      "Sensor components are not supported on the '%s' interface type, but they were defined.",
      info_.hardware_plugin_name.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (!info_.gpios.empty()) {
    RCLCPP_FATAL(
      get_logger(),
      "GPIO components are not supported on the '%s' interface type, but they were defined.",
      info_.hardware_plugin_name.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  // NOTE(SuperJappie08): Theoretically there could also be a interface which listens to multiple encoders/devices.
  //                      However, that makes configuration less clear and the code more complex.
  if (info_.joints.size() != 1) {
    RCLCPP_FATAL(
      get_logger(), "Exactly 1 Joint is expected on the '%s' interface type, but %zu were defined.",
      info_.hardware_plugin_name.c_str(), info_.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  auto joint = info_.joints[0];

  // Check if the specified joint is consistent with the capabilities of this hardware interface.
  if (!joint.state_interfaces.empty()) {
    RCLCPP_FATAL(
      get_logger(), "The '%s' interface type does not support any state interfaces.",
      info_.hardware_plugin_name.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (joint.is_mimic == hardware_interface::MimicAttribute::TRUE) {
    // TODO(SuperJappie08): Figure out if supporting mimic joints make sense?
    RCLCPP_FATAL(
      get_logger(), "Mimic joints are currently not supported on '%s' interface types.",
      info_.hardware_plugin_name.c_str());
    return CallbackReturn::ERROR;
  }

  if (joint.command_interfaces.size() == 1) {
    auto find_interface = [joint](auto interface_name) {
      return std::find_if(
        joint.command_interfaces.cbegin(), joint.command_interfaces.cend(),
        [interface_name](auto iter) { return iter.name == interface_name; });
    };

    // if (auto position_interface = find_interface(hardware_interface::HW_IF_POSITION);
    //     position_interface != joint.command_interfaces.cend()) {
    //   // FIXME(SuperJappie08): Check position interface
    // } else {
    //   RCLCPP_FATAL(
    //     get_logger(),
    //     "Joint '%s' of hardware interface '%s' [%s] has no 'position' command interface defined.",
    //     joint.name.c_str(), info_.name.c_str(), info_.hardware_plugin_name.c_str());
    //   return hardware_interface::CallbackReturn::ERROR;
    // }

    if (auto velocity_interface = find_interface(hardware_interface::HW_IF_VELOCITY);
        velocity_interface != joint.command_interfaces.cend()) {
      // FIXME(SuperJappie08): Check velocity interface
    } else {
      RCLCPP_FATAL(
        get_logger(),
        "Joint '%s' of hardware interface '%s' [%s] has no 'velocity' command interface defined.",
        joint.name.c_str(), info_.name.c_str(), info_.hardware_plugin_name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
  } else {
    RCLCPP_FATAL(
      get_logger(),
      "Joint '%s' of hardware interface '%s' [%s] has a unexpected amount of command interfaces. "
      "Expected 1 ['velocity'], but found %zu interfaces.",
      joint.name.c_str(), info_.name.c_str(), info_.hardware_plugin_name.c_str(),
      joint.command_interfaces.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  // FIXME(SuperJappie08): Implement everything

  auto node_options =
    rclcpp::NodeOptions().start_parameter_event_publisher(false).start_parameter_services(false);
  node_ = rclcpp::Node::make_shared(MOTOR_ACTUATOR_NODE_NAME_PREFIX + get_name(), node_options);

  // FIXME(SuperJappie08): Check if QoS makes sense when only sending updates?
  speed_publisher_ = node_->create_publisher<SpeedMsg>(topic_name, rclcpp::SensorDataQoS());
  speed_publisher_rt_.reset(new realtime_tools::RealtimePublisher<SpeedMsg>(speed_publisher_));

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MotorActuator::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  for (auto joint_command : joint_commands_) {
    if (joint_command->get_interface_name() == hardware_interface::HW_IF_VELOCITY) {
      // TODO(SuperJappie08): Possibly make initial command velocity configurable.
      if (!joint_command->set_value(0.0)) {
        RCLCPP_ERROR(
          get_logger(), "Failed to set initial value for command interface '%s'.",
          joint_command->get_name().c_str());
        return hardware_interface::CallbackReturn::FAILURE;
      }
    } else [[unlikely]] {
      RCLCPP_ERROR(
        get_logger(),
        "Unexpected command interface type '%s' found. (full command interface: '%s')",
        joint_command->get_interface_name().c_str(), joint_command->get_name().c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  // FIXME(SuperJappie08): Implement everything

  // FIXME(SuperJappie08): Make this untmp

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MotorActuator::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // FIXME(SuperJappie08): Implement everything

  // Make sure the speed is 0 when deactivated
  speed_publisher_rt_->lock();
  speed_publisher_rt_->msg_.data = 0;
  speed_publisher_rt_->unlockAndPublish();

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type MotorActuator::perform_command_mode_switch(
  const std::vector<std::string> & /*start_interfaces*/,
  const std::vector<std::string> & stop_interfaces)
{
  // NOTE(SuperJappie08): This prevents endless spinning motors when the controller is changed.

  // TODO(SuperJappie08): When switching controllers (deactivate A, activate B) will send an intermediate 0.0 velocity to the motors.
  //                      Is this an issue?
  for (auto stopped_cmd_name : stop_interfaces) {
    if (auto joint_command = std::find_if(
          joint_commands_.begin(), joint_commands_.end(),
          [stopped_cmd_name](auto iter) { return iter->get_name() == stopped_cmd_name; });
        joint_command != joint_commands_.end()) {
      if (!joint_command->get()->set_value(0.0)) {
        RCLCPP_ERROR(
          get_logger(), "Failed to set value for command interface '%s'.",
          joint_command->get()->get_name().c_str());
        return hardware_interface::return_type::ERROR;
      }
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type MotorActuator::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*duration*/)
{
  // FIXME(SuperJappie08): Implement everything
  // TODO(SuperJappie08): Maybe add optional current command speed for diff drive controller
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type MotorActuator::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*duration*/)
{
  if (get_lifecycle_state().label() != hardware_interface::lifecycle_state_names::ACTIVE) {
    return hardware_interface::return_type::OK;
  }

  for (auto joint_command : joint_commands_) {
    if (joint_command->get_interface_name() == hardware_interface::HW_IF_VELOCITY) [[likely]] {
      auto commanded_velocity = joint_command->get_optional();

      // Silently continue if the speed cannot be published, assume controller frequency is high enough
      if (commanded_velocity.has_value() && speed_publisher_rt_->trylock()) {
        auto data = (int)(commanded_velocity.value() / max_motor_speed_ * 100.0);

        auto & msg = speed_publisher_rt_->msg_;

        // TODO(SuperJappie08): Maybe add an interval based publisher, so that it will be published if the last message was a while ago.

        // Only publish when the command velocity has changed.
        // TODO(SuperJappie08): Figure out if QoS as sensor makes sense with this being enabled.
        if (msg.data != data) {
          msg.data = data;
          speed_publisher_rt_->unlockAndPublish();
        } else {
          speed_publisher_rt_->unlock();
        }
      }

      continue;
    }

    RCLCPP_WARN(
      get_logger(), "Retrieving the value of command interface '%s' failed",
      joint_command->get_name().c_str());
  }

  // FIXME(SuperJappie08): Implement everything
  return hardware_interface::return_type::OK;
}

}  // namespace mirte_modular_hardware

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(mirte_modular_hardware::MotorActuator, hardware_interface::ActuatorInterface)
