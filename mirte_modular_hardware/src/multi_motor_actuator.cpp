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

#include <cmath>
#include <limits>
/* FIXME(SuperJappie08): TO SEPERATE INCLUDES */
#include <hardware_interface/actuator_interface.hpp>
#include <hardware_interface/lexical_casts.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <hardware_interface/types/lifecycle_state_names.hpp>
/* FIXME(SuperJappie08): TO SEPERATE INCLUDES */
#include <rclcpp/logging.hpp>
#include <rclcpp_lifecycle/state.hpp>

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

  if (auto topic_pair = info_.hardware_parameters.find("topic");
      topic_pair != info_.hardware_parameters.end()) {
    RCLCPP_INFO(
      get_logger(), "Using '%s' as the topic name (relative to the hardware node).",
      topic_pair->second.c_str());
    topic_name_ = topic_pair->second;
  } else {
    RCLCPP_FATAL(
      get_logger(),
      "Missing the required 'topic' hardware parameter, "
      "to indicate the topic (relative to the the hardware node).");
    return hardware_interface::CallbackReturn::ERROR;
  }

  // TODO(SuperJappie08): Retrieve general params.

  // Optional hardware parameters
  // TODO(SuperJappie08): Retrieve optional params

  // Validate if the configuration is valid.

  if (!info_.transmissions.empty()) {
    RCLCPP_FATAL(
      get_logger(),
      "Transmissions are not supported on the '%s' interface type, but they were defined.",
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
        if (velocity_interface->max.empty()) {
          RCLCPP_FATAL(
            get_logger(),
            "Missing the required '%s/velocity.max' hardware parameter [double, rad/s]. "
            "This is used in the conversion to the percentage based command speed.",
            joint.name.c_str());
          return hardware_interface::CallbackReturn::ERROR;
        }
        auto max_motor_speed = hardware_interface::stod(velocity_interface->max);

        // Check if the min bounds are defined/symmetric
        if (!velocity_interface->min.empty()) {
          auto min_motor_speed = hardware_interface::stod(velocity_interface->min);
          if (
            std::fabs(std::fabs(max_motor_speed) - std::fabs(min_motor_speed)) >
            std::numeric_limits<double>::epsilon()) {
            // TODO(SuperJappie08): Could consider taking the biggest absolute value as the conversion factor.
            RCLCPP_FATAL(
              get_logger(),
              "Both the 'velocity.min' and 'velocity.max' hardware parameters are defined for '%s' "
              "[double, rad/s]. They are non-symmetric, which is unsupported. "
              "(They must be equal in magnitude!) (Recieved '%f' (min) and '%f' (max))",
              joint.name.c_str(), min_motor_speed, max_motor_speed);
            return hardware_interface::CallbackReturn::ERROR;
          }
        }

        auto interface_name = joint.name + "/" + velocity_interface->name;
        auto tmx_motor_name = retrieve_tmx_motor_name(joint);
        motor_handles_.push_back({
          .max_speed = max_motor_speed,
          .interface_name = interface_name,
          .tmx_motor_name = tmx_motor_name,
          .idx = (int)motor_handles_.size(),
        });
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
  }

  // TODO(SuperJappie08): This might need to be moved to activate
  // Setup the communication
  auto node_options =
    rclcpp::NodeOptions().start_parameter_event_publisher(false).start_parameter_services(false);
  node_ =
    rclcpp::Node::make_shared(MULTI_MOTOR_ACTUATOR_NODE_NAME_PREFIX + get_name(), node_options);

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MultiMotorActuator::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // FIXME(SuperJappie08): Check if QoS makes sense when only sending updates?
  multi_speed_publisher_ =
    get_node()->create_publisher<MultiSpeedMsg>(topic_name_, rclcpp::SensorDataQoS());
  multi_speed_publisher_rt_.reset(
    new realtime_tools::RealtimePublisher<MultiSpeedMsg>(multi_speed_publisher_));

  if (multi_speed_publisher_->get_subscription_count() == 0) {
    RCLCPP_WARN(
      get_logger(), "No subscribers on multi motor topic '%s' yet! Is it the correct topic?",
      topic_name_.c_str());
  }

  multi_speed_publisher_rt_->lock();
  bool send_initial_command = false;
  for (const auto & motor_handle : motor_handles_) {
    // When no initial value is specified NAN is used, which we convert to 0.0.
    // So if it is not NAN an initial value was specified.
    if (std::isnan(get_command(motor_handle.interface_name))) {
      set_command(motor_handle.interface_name, 0.0);
    } else {
      send_initial_command = true;
    }
    NamedSpeed named_speed;
    named_speed.name = motor_handle.tmx_motor_name;
    named_speed.speed =
      motor_handle.calculate_motor_value(get_command(motor_handle.interface_name));

    multi_speed_publisher_rt_->msg_.speeds.push_back(named_speed);
  }
  if (send_initial_command) {
    multi_speed_publisher_rt_->unlockAndPublish();
  } else {
    multi_speed_publisher_rt_->unlock();
  }

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
  for (auto & speed : multi_speed_publisher_rt_->msg_.speeds) {
    speed.speed = 0;
  }
  multi_speed_publisher_rt_->unlockAndPublish();

  for (const auto & motor_handle : motor_handles_) {
    set_command(motor_handle.interface_name, 0.0);
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type MultiMotorActuator::perform_command_mode_switch(
  const std::vector<std::string> & /*start_interfaces*/,
  const std::vector<std::string> & stop_interfaces)
{
  // NOTE(SuperJappie08): This prevents endless spinning motors when the controller is changed.

  // NOTE(SuperJappie08): When switching controllers (deactivate A, activate B) will send an intermediate 0.0 velocity to the motors.
  //                      Most controllers reset their state on deactivation, so it might not be worth the effort
  for (const auto & stopped_cmd_name : stop_interfaces) {
    if (const auto & motor_handle = std::find_if(
          motor_handles_.cbegin(), motor_handles_.cend(),
          [stopped_cmd_name](auto iter) { return iter.interface_name == stopped_cmd_name; });
        motor_handle != motor_handles_.cend()) {
      set_command(motor_handle->interface_name, 0.0);
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

  if (multi_speed_publisher_rt_->trylock()) [[likely]] {
    auto & msg = multi_speed_publisher_rt_->msg_;
    auto changed = false;

    for (const auto & motor_handle : motor_handles_) {
      auto commanded_velocity = get_command(motor_handle.interface_name);

      auto cmd_vel_raw = commanded_velocity;
      auto cmd_vel = (std::isfinite(cmd_vel_raw)) ? cmd_vel_raw : 0.0;
      auto data = motor_handle.calculate_motor_value(cmd_vel);

      if (msg.speeds[motor_handle.idx].speed != data) {
        msg.speeds[motor_handle.idx].speed = data;
        changed = true;
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
