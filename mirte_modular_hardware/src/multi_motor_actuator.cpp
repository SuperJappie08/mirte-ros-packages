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
#include <cmath>
#include <hardware_interface/actuator_interface.hpp>
#include <hardware_interface/lexical_casts.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <hardware_interface/types/lifecycle_state_names.hpp>
#include <joint_limits/joint_limits.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp_lifecycle/state.hpp>
/* FIXME(SuperJappie08): TO SEPERATE INCLUDES */

#include "mirte_modular_hardware/multi_motor_actuator.hpp"

namespace mirte_modular_hardware
{
const std::string MULTI_MOTOR_ACTUATOR_NODE_NAME_PREFIX =
  "mirte_modular_hardware_multi_motor_actuator_";

hardware_interface::CallbackReturn MultiMotorActuator::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::ActuatorInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Retrieve general parameters

  /* NOTE(SuperJappie08): The initial implementation will support a maximum of 4 motors.
    However in the future this could be expanded and motor groups could be created (with max 4 motors per group).
  */

  // FIXME(SuperJappie08): Implement everything

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

  // TODO(SuperJappie08): Consider making this motor specific or per motor override
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

  if (info_.joints.empty()) {
    RCLCPP_FATAL(
      get_logger(),
      "Atleast one joint component is required on the '%s' interface type, but none where defined.",
      info_.hardware_plugin_name.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (info_.joints.size() > 4) {
    /* NOTE(SuperJappie08): This is a limit of the current implementation,
      could introduce 'joint groups' which are motors which will be grouped. */
    RCLCPP_FATAL(
      get_logger(), "The '%s' interface type supports atmost 4 joints, but %ld where provided",
      info_.hardware_plugin_name.c_str(), info_.joints.size());
  }

  for (auto joint : info_.joints) {
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

    // TODO(SuperJappie08): Check if local motor_name parameter is valid
    // TODO(SuperJappie08): Check if local max_motor_speed parameter is valid

    joint_tmx_map_.insert({joint.name, retrieve_tmx_motor_name(joint)});
  }

  // TODO(SuperJappie08): This might need to be moved to activate
  // Setup the communication
  auto node_options =
    rclcpp::NodeOptions().start_parameter_event_publisher(false).start_parameter_services(false);
  node_ =
    rclcpp::Node::make_shared(MULTI_MOTOR_ACTUATOR_NODE_NAME_PREFIX + get_name(), node_options);

  // FIXME(SuperJappie08): Check if QoS makes sense when only sending updates?
  multi_speed_publisher_ =
    node_->create_publisher<MultiSpeedMsg>(topic_name, rclcpp::SensorDataQoS());
  multi_speed_publisher_rt_.reset(
    new realtime_tools::RealtimePublisher<MultiSpeedMsg>(multi_speed_publisher_));

  multi_speed_publisher_rt_->lock();
  for (const auto & [joint_name, tmx_motor_name] : joint_tmx_map_) {
    NamedSpeed named_speed;
    named_speed.name = tmx_motor_name;
    named_speed.speed = 0;

    joint_idx_map_.insert({joint_name, multi_speed_publisher_rt_->msg_.speeds.size()});
    multi_speed_publisher_rt_->msg_.speeds.push_back(named_speed);
  }
  multi_speed_publisher_rt_->unlock();

  return hardware_interface::CallbackReturn::SUCCESS;
}

// TODO(SuperJappie08): Consider adding a on_activate, which calls a service to verify the available motors.
// hardware_interface::CallbackReturn MultiMotorActuator::on_activate(
//   const rclcpp_lifecycle::State & /*previous_state*/
// )
// {
//   for (auto joint_command : joint_commands_) {
//     if (joint_command->get_interface_name() == hardware_interface::HW_IF_VELOCITY) {
//       // TODO(SuperJappie08): Possibly make initial command velocity configurable.
//       if (!joint_command->set_value(0.0)) {
//         RCLCPP_ERROR(
//           get_logger(), "Failed to set initial value for command interface '%s'.",
//           joint_command->get_name().c_str());
//         return hardware_interface::CallbackReturn::FAILURE;
//       }
//     } else [[unlikely]] {
//       RCLCPP_ERROR(
//         get_logger(),
//         "Unexpected command interface type '%s' found. (full command interface: '%s')",
//         joint_command->get_interface_name().c_str(), joint_command->get_name().c_str());
//       return hardware_interface::CallbackReturn::ERROR;
//     }
//   }
// }

hardware_interface::CallbackReturn MultiMotorActuator::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // FIXME(SuperJappie08): Implement Setting speed to 0? (probably taken care of by perform_command_mode_switch)

  // Make sure the speed is 0 when deactivated
  multi_speed_publisher_rt_->lock();
  for (auto speed : multi_speed_publisher_rt_->msg_.speeds) {
    speed.speed = 0;
  }
  multi_speed_publisher_rt_->unlockAndPublish();

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type MultiMotorActuator::perform_command_mode_switch(
  const std::vector<std::string> & /*start_interfaces*/,
  const std::vector<std::string> & stop_interfaces)
{
  // NOTE(SuperJappie08): This prevents endless spinning motors when the controller is changed.

  // TODO(SuperJappie08): When switching controllers (deactivate A, activate B) will send an intermediate 0.0 velocity to the motors.
  //                      Is this an issue?
  for (auto stopped_cmd_name : stop_interfaces) {
    // TODO(SuperJappie08): Consider stop->start switch (do reset?)
    // if (auto start_cmd_name = std::find_if(
    //       start_interfaces.cbegin(), start_interfaces.cend(),
    //       [stopped_cmd_name](auto interface) { return interface == stopped_cmd_name; });
    //     start_cmd_name != start_interfaces.cend()) {
    //   // If the command interface switches controller, the speed doesn't have to be reset.
    //   continue;
    // }

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

hardware_interface::return_type MultiMotorActuator::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*duration*/)
{
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type MultiMotorActuator::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*duration*/)
{
  if (get_lifecycle_state().label() != hardware_interface::lifecycle_state_names::ACTIVE) {
    return hardware_interface::return_type::OK;
  }

  // FIXME(SuperJappie08): Add something with structs, to prevent name lookup in RT-loop

  if (multi_speed_publisher_rt_->trylock()) [[likely]] {
    auto & msg = multi_speed_publisher_rt_->msg_;
    auto changed = false;

    for (auto joint_command : joint_commands_) {
      if (joint_command->get_interface_name() == hardware_interface::HW_IF_VELOCITY) [[likely]] {
        auto commanded_velocity = joint_command->get_optional();

        if (commanded_velocity.has_value()) {
          auto cmd_vel_raw = commanded_velocity.value();
          auto cmd_vel = (std::isfinite(cmd_vel_raw)) ? cmd_vel_raw : 0.0;
          auto data = (int)(cmd_vel / max_motor_speed_ * 100.0);

          if (!joint_idx_map_.contains(joint_command->get_prefix_name())) {
            RCLCPP_ERROR(
              get_logger(), "Unknown joint '%s' encountered. Ignoring!",
              joint_command->get_name().c_str());
            continue;
          }

          auto idx = joint_idx_map_[joint_command->get_prefix_name()];
          if (msg.speeds[idx].speed != data) {
            msg.speeds[idx].speed = data;
            changed = true;
          }
        }
      }
    }

    if (changed) {
      multi_speed_publisher_rt_->unlockAndPublish();
    } else {
      multi_speed_publisher_rt_->unlock();
    }
  } else {
    RCLCPP_WARN(get_logger(), "Unable to lock publisher");
  }

  // FIXME(SuperJappie08): Implement
  return hardware_interface::return_type::OK;
}

const std::string & MultiMotorActuator::retrieve_tmx_motor_name(
  const hardware_interface::ComponentInfo & joint) const
{
  if (auto motor_name_iter = joint.parameters.find("motor_name");
      motor_name_iter != joint.parameters.cend() && !motor_name_iter->second.empty()) {
    return motor_name_iter->second;
  }
  return joint.name;
}

}  // namespace mirte_modular_hardware

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  mirte_modular_hardware::MultiMotorActuator, hardware_interface::ActuatorInterface)
