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

#include <chrono>
#include <memory>
#include <numbers>
#include <thread>
/* FIXME(SuperJappie08): TO SEPERATE INCLUDES */
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/lexical_casts.hpp>
#include <hardware_interface/lifecycle_helpers.hpp>
#include <hardware_interface/sensor_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <hardware_interface/types/lifecycle_state_names.hpp>
/* FIXME(SuperJappie08): TO SEPERATE INCLUDES */
#include <rclcpp/context.hpp>
#include <rclcpp/executor.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/wait_for_message.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include "mirte_modular_hardware/encoder_sensor.hpp"
#include "mirte_msgs/msg/encoder.hpp"

namespace mirte_modular_hardware {

/* FIXME(SuperJappie08): Consider allowing the hardware nodes to be put in a
  (hidden) namespace. Then for simplicity the topic would need to be defined as
  from the controller_manager. */

const std::string ENCODER_SENSOR_NODE_NAME_PREFIX =
    "mirte_modular_hardware_encoder_sensor_";

hardware_interface::CallbackReturn
EncoderSensor::on_init(const hardware_interface::HardwareInfo &info) {
  if (hardware_interface::SensorInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Retrieve general parameters

  if (auto ticks_per_rotation_raw =
          info_.hardware_parameters.find("ticks_per_rotation");
      ticks_per_rotation_raw != info_.hardware_parameters.end()) {
    // TODO(SuperJappie08): Possible store this as rad per tick
    ticks_per_rotation_ =
        hardware_interface::stod(ticks_per_rotation_raw->second);
    RCLCPP_INFO(get_logger(), "Loaded 'ticks_per_rotation' [%f]",
                ticks_per_rotation_);
  } else {
    RCLCPP_FATAL(get_logger(), "Missing required 'ticks_per_rotation' hardware "
                               "parameter, to indicate the number of encoder "
                               "ticks corresponding to a full rotation.");
    return hardware_interface::CallbackReturn::ERROR;
  }

  std::string encoder_topic;
  if (auto encoder_topic_pair = info_.hardware_parameters.find("topic");
      encoder_topic_pair != info_.hardware_parameters.end()) {
    RCLCPP_INFO(get_logger(),
                "Using '%s' as the topic name (relative to the hardware node).",
                encoder_topic_pair->second.c_str());
    encoder_topic = encoder_topic_pair->second;
  } else {
    // TODO(SuperJappie08): Consider making this parameter optional.
    RCLCPP_FATAL(get_logger(), "Missing required 'topic' hardware parameter, "
                               "to indicate the topic (relative to the "
                               "the hardware node).");
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Optional hardware parameters
  if (auto initial_message_timeout =
          info_.hardware_parameters.find("initial_message_timeout_ms");
      initial_message_timeout != info_.hardware_parameters.end()) {
    RCLCPP_INFO(get_logger(),
                "Attempting to parse 'initial_message_timeout_ms' parameter "
                "[int(in milliseconds)]. (use -1 "
                "to disable timeout)");
    initial_message_timeout_ =
        std::chrono::milliseconds(std::stol(initial_message_timeout->second));
  }

  if (initial_message_timeout_ == std::chrono::milliseconds(-1)) {
    RCLCPP_WARN(get_logger(), "The initial message timeout has been disabled, "
                              "controller could wait indefinitely.");
  } else {
    RCLCPP_INFO_STREAM(get_logger(), "Using initial message timeout of "
                                         << initial_message_timeout_ << ".");
  }

  // Validate if the configuration is valid.

  if (!info_.transmissions.empty()) {
    RCLCPP_FATAL(get_logger(),
                 "Transmission components are not supported on the '%s' "
                 "interface type, but they were "
                 "defined.",
                 info_.hardware_plugin_name.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (!info_.sensors.empty()) {
    RCLCPP_FATAL(get_logger(),
                 "Sensor components are not supported on the '%s' interface "
                 "type, but they were defined.",
                 info_.hardware_plugin_name.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (!info_.gpios.empty()) {
    RCLCPP_FATAL(get_logger(),
                 "GPIO components are not supported on the '%s' interface "
                 "type, but they were defined.",
                 info_.hardware_plugin_name.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  // NOTE(SuperJappie08): Theoretically there could also be a interface which
  // listens to multiple encoders/devices.
  //                      However, that makes configuration less clear and the
  //                      code more complex.
  if (info_.joints.size() != 1) {
    RCLCPP_FATAL(get_logger(),
                 "Exactly 1 Joint is expected on the '%s' interface type, but "
                 "%zu were defined.",
                 info_.hardware_plugin_name.c_str(), info_.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  auto joint = info_.joints[0];

  // Check if the specified joint is consistent with the capabilities of this
  // hardware interface.
  if (!joint.command_interfaces.empty()) {
    RCLCPP_FATAL(
        get_logger(),
        "The '%s' interface type does not support any command interfaces.",
        info_.hardware_plugin_name.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (joint.is_mimic == hardware_interface::MimicAttribute::TRUE) {
    // TODO(SuperJappie08): Figure out if supporting mimic joints make sense?
    RCLCPP_FATAL(
        get_logger(),
        "Mimic joints are currently not supported on '%s' interface types.",
        info_.hardware_plugin_name.c_str());
    return CallbackReturn::ERROR;
  }

  if (joint.state_interfaces.size() == 2) {
    auto find_interface = [joint](auto interface_name) {
      return std::find_if(
          joint.state_interfaces.cbegin(), joint.state_interfaces.cend(),
          [interface_name](auto iter) { return iter.name == interface_name; });
    };

    if (auto position_interface =
            find_interface(hardware_interface::HW_IF_POSITION);
        position_interface != joint.state_interfaces.cend()) {
      // FIXME(SuperJappie08): Check position interface
    } else {
      RCLCPP_FATAL(get_logger(),
                   "Joint '%s' of hardware interface '%s' [%s] has no "
                   "'position' state interface defined.",
                   joint.name.c_str(), info_.name.c_str(),
                   info_.hardware_plugin_name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (auto velocity_interface =
            find_interface(hardware_interface::HW_IF_VELOCITY);
        velocity_interface != joint.state_interfaces.cend()) {
      // FIXME(SuperJappie08): Check velocity interface
    } else {
      // TODO(SuperJappie08): Consider making the velocity interface optional.

      RCLCPP_FATAL(get_logger(),
                   "Joint '%s' of hardware interface '%s' [%s] has no "
                   "'velocity' state interface defined.",
                   joint.name.c_str(), info_.name.c_str(),
                   info_.hardware_plugin_name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
  } else {
    RCLCPP_FATAL(
        get_logger(),
        "Joint '%s' of hardware interface '%s' [%s] has a unexpected amount of "
        "state interfaces. "
        "Expected 2 ['position', 'velocity'], but found %zu interfaces.",
        joint.name.c_str(), info_.name.c_str(),
        info_.hardware_plugin_name.c_str(), joint.state_interfaces.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  // TODO(SuperJappie08): possibly verify Joint parameters

  // TODO(SuperJappie08): Process general Parameters

  // TODO(SuperJappie08): Setup initialize encoder communication

  // TODO: Check configuration of interfaces in urdf

  auto node_options = rclcpp::NodeOptions()
                          .start_parameter_event_publisher(false)
                          .start_parameter_services(false);
  node_ = rclcpp::Node::make_shared(
      ENCODER_SENSOR_NODE_NAME_PREFIX + get_name(), node_options);

  // TODO(SuperJappie08): Investigate if a single message buffer (1 msg) could
  // be used if the previous position state is used to calculate the speed.
  //                    - Pros: Less confusing and coping, out-zeroing when
  //                    crashed
  //                    - Cons: If message arrive late it zeros the velocity,
  //                    could introduce chatter
  encoder_subscriber_ = node_->create_subscription<mirte_msgs::msg::Encoder>(
      // FIXME(SuperJappie08): Figure out if keep_last(5) (default) or
      // keep_last(1/2/3) is better
      encoder_topic, rclcpp::SensorDataQoS() /* .keep_last(1)*/,
      [this](const EncoderMsg::ConstSharedPtr msg) {
        this->latest_msgs_.writeFromNonRT(
            {msg, this->latest_msgs_.readFromNonRT()->first});
      });

  executor_ = rclcpp::executors::SingleThreadedExecutor::make_shared();
  executor_thread_.reset(
      new std::thread(std::bind(&rclcpp::Executor::spin, executor_)));

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn EncoderSensor::on_configure(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  // Initialize the buffer, so the initial read will also be valid
  auto context = node_->get_node_options().context();

  RCLCPP_INFO(get_logger(), "Waiting for first two messages on '%s'",
              encoder_subscriber_->get_topic_name());

  EncoderMsg first_msg;
  if (!rclcpp::wait_for_message(first_msg, encoder_subscriber_, context,
                                initial_message_timeout_)) {
    /* NOTE(SuperJappie08): Return failure since then we would revert to
       unconfigured (if started from a non configured state). If the hardware
       interface fails when start requires a state ros2_control crashes
       (https://github.com/ros-controls/ros2_control/issues/2290) */
    RCLCPP_ERROR_STREAM(get_logger(),
                        "Timed out waiting for first Encoder message on '"
                            << encoder_subscriber_->get_topic_name()
                            << "'. [Timeout = " << initial_message_timeout_
                            << "]");
    return hardware_interface::CallbackReturn::FAILURE;
  }

  EncoderMsg second_msg;
  if (!rclcpp::wait_for_message(second_msg, encoder_subscriber_, context,
                                initial_message_timeout_)) {
    RCLCPP_ERROR_STREAM(get_logger(),
                        "Timed out waiting for second Encoder message on '"
                            << encoder_subscriber_->get_topic_name()
                            << "'. [Timeout = " << initial_message_timeout_
                            << "] (hint: Check the frequency of '"
                            << encoder_subscriber_->get_topic_name() << "')");
    return hardware_interface::CallbackReturn::FAILURE;
  }

  RCLCPP_INFO(get_logger(), "Recieved the intial two messages on '%s'",
              encoder_subscriber_->get_topic_name());

  latest_msgs_.initRT({std::make_shared<const EncoderMsg>(second_msg),
                       std::make_shared<const EncoderMsg>(first_msg)});

  executor_->add_node(node_);

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
EncoderSensor::on_cleanup(const rclcpp_lifecycle::State & /*previous_state*/) {
  executor_->remove_node(node_);

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
EncoderSensor::on_shutdown(const rclcpp_lifecycle::State &previous_state) {
  if (previous_state.label() ==
      hardware_interface::lifecycle_state_names::UNKNOWN) {
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  if (previous_state.label() !=
      hardware_interface::lifecycle_state_names::UNCONFIGURED) {
    executor_->remove_node(node_);
  }

  // In states INACTIVE and ACTIVE the executor is running
  stop_executor();

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
EncoderSensor::on_error(const rclcpp_lifecycle::State &previous_state) {
  auto label = previous_state.label();
  if (label == hardware_interface::lifecycle_state_names::ACTIVE ||
      label == hardware_interface::lifecycle_state_names::INACTIVE) {
    executor_->remove_node(node_);
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type
EncoderSensor::read(const rclcpp::Time & /*time*/,
                    const rclcpp::Duration & /*period*/) {
  auto newest_msg = latest_msgs_.readFromRT()->first;
  auto older_msg = latest_msgs_.readFromRT()->second;

  if (!newest_msg || !older_msg) {
    RCLCPP_ERROR(get_logger(), "There are no msgs");
    return hardware_interface::return_type::ERROR;
  }

  for (auto joint_state : joint_states_) {
    if (joint_state->get_interface_name() ==
        hardware_interface::HW_IF_POSITION) {
      double position = ((double)newest_msg->value) / ticks_per_rotation_ *
                        std::numbers::pi * 2.0;

      if (joint_state->set_value(position)) [[likely]] {
        continue;
      }
    } else if (joint_state->get_interface_name() ==
               hardware_interface::HW_IF_VELOCITY) {
      double difference = ((double)(newest_msg->value - older_msg->value)) /
                          ticks_per_rotation_ * std::numbers::pi * 2.0;
      // FIXME(SuperJappie08): It works with senconds?
      auto dt = (rclcpp::Time(newest_msg->header.stamp) -
                 rclcpp::Time(older_msg->header.stamp))
                    .seconds();
      double velocity = difference / dt;

      if (joint_state->set_value(velocity)) [[likely]] {
        continue;
      }
    } else [[unlikely]] {
      RCLCPP_WARN_ONCE(get_logger(), "Unimplemented state_interface: %s",
                       joint_state->get_name().c_str());
      return hardware_interface::return_type::ERROR;
    }

    RCLCPP_WARN(get_logger(),
                "Setting the value of state interface '%s' failed",
                joint_state->get_name().c_str());
  }

  return hardware_interface::return_type::OK;
}

void EncoderSensor::stop_executor() noexcept {
  if (executor_ && executor_->is_spinning()) {
    executor_->cancel();
    if (executor_thread_ && executor_thread_->joinable()) [[likely]] {
      executor_thread_->join();
    }
  }
}

} // namespace mirte_modular_hardware

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(mirte_modular_hardware::EncoderSensor,
                       hardware_interface::SensorInterface)
