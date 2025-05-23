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

#include <memory>
#include <numbers>
#include <thread>
/* FIXME(SuperJappie08): TO SEPERATE INCLUDES */
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/lexical_casts.hpp>
#include <hardware_interface/lifecycle_helpers.hpp>
#include <hardware_interface/sensor_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
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

namespace mirte_modular_hardware
{

/* FIXME(SuperJappie08): Consider allowing the hardware nodes to be put in a (hidden) namespace.
  Then for simplicity the topic would need to be defined as from the controller_manager. */

const std::string ENCODER_SENSOR_NODE_NAME_PREFIX = "mirte_modular_hardware_encoder_sensor_";

hardware_interface::CallbackReturn EncoderSensor::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SensorInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Retrieve general parameters

  if (auto ticks_per_rotation_raw = info_.hardware_parameters.find("ticks_per_rotation");
      ticks_per_rotation_raw != info_.hardware_parameters.end()) {
    // TODO(SuperJappie08): Possible store this as rad per tick
    ticks_per_rotation_ = hardware_interface::stod(ticks_per_rotation_raw->second);
    RCLCPP_INFO(get_logger(), "Loaded 'ticks_per_rotation' [%f]", ticks_per_rotation_);
  } else {
    RCLCPP_FATAL(
      get_logger(),
      "Missing required 'ticks_per_rotation' hardware parameter, to indicate the number of encoder "
      "ticks corresponding to a full rotation.");
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (auto encoder_topic = info_.hardware_parameters.find("topic");
      encoder_topic != info_.hardware_parameters.end()) {
    RCLCPP_INFO(
      get_logger(), "Using '%s' as the topic name (relative to the hardware node).",
      encoder_topic->second.c_str());
    encoder_topic_ = encoder_topic->second;
  } else {
    // TODO(SuperJappie08): Consider making this parameter optional.
    RCLCPP_FATAL(
      get_logger(),
      "Missing required 'topic' hardware parameter, to indicate the topic (relative to the "
      "the hardware node).");
    return hardware_interface::CallbackReturn::ERROR;
  }

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
  if (!joint.command_interfaces.empty()) {
    RCLCPP_FATAL(
      get_logger(), "The '%s' interface type does not support any command interfaces.",
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

  if (joint.state_interfaces.size() == 2) {
    auto find_interface = [joint](auto interface_name) {
      return std::find_if(
        joint.state_interfaces.cbegin(), joint.state_interfaces.cend(),
        [interface_name](auto iter) { return iter.name == interface_name; });
    };

    if (auto position_interface = find_interface(hardware_interface::HW_IF_POSITION);
        position_interface != joint.state_interfaces.cend()) {
      // FIXME(SuperJappie08): Check position interface
    } else {
      RCLCPP_FATAL(
        get_logger(),
        "Joint '%s' of hardware interface '%s' [%s] has no 'position' state interface defined.",
        joint.name.c_str(), info_.name.c_str(), info_.hardware_plugin_name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (auto velocity_interface = find_interface(hardware_interface::HW_IF_VELOCITY);
        velocity_interface != joint.state_interfaces.cend()) {
      // FIXME(SuperJappie08): Check velocity interface
    } else {
      // TODO(SuperJappie08): Consider making the velocity interface optional.

      RCLCPP_FATAL(
        get_logger(),
        "Joint '%s' of hardware interface '%s' [%s] has no 'velocity' state interface defined.",
        joint.name.c_str(), info_.name.c_str(), info_.hardware_plugin_name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
  } else {
    RCLCPP_FATAL(
      get_logger(),
      "Joint '%s' of hardware interface '%s' [%s] has a unexpected amount of state interfaces. "
      "Expected 2 ['position', 'velocity'], but found %zu interfaces.",
      joint.name.c_str(), info_.name.c_str(), info_.hardware_plugin_name.c_str(),
      joint.state_interfaces.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  // TODO(SuperJappie08): possibly verify Joint parameters

  // TODO(SuperJappie08): Process general Parameters

  // TODO(SuperJappie08): Setup initialize encoder communication

  // TODO: Check configuration of interfaces in urdf

  // FIXME(SuperJappie08): Figure out if the executor (thread) should be started here? Since on_init should 'initialize containers and member variables'
  //                       In that case keep the thread and executor arround until finalization

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn EncoderSensor::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  auto node_options =
    rclcpp::NodeOptions().start_parameter_event_publisher(false).start_parameter_services(false);
  node_ = rclcpp::Node::make_shared(ENCODER_SENSOR_NODE_NAME_PREFIX + get_name(), node_options);

  // TODO(SuperJappie08): Investigate if a single message buffer (1 msg) could be used if the previous position state is used to calculate the speed.
  //                    - Pros: Less confusing and coping, out-zeroing when crashed
  //                    - Cons: If message arrive late it zeros the velocity, could introduce chatter
  encoder_subscriber_ = node_->create_subscription<mirte_msgs::msg::Encoder>(
    // FIXME(SuperJappie08): Figure out if keep_last(5) (default) or keep_last(1/2/3) is better
    encoder_topic_, rclcpp::SensorDataQoS() /* .keep_last(1)*/,
    [this](const EncoderMsg::ConstSharedPtr msg) {
      this->latest_msgs_.writeFromNonRT({msg, this->latest_msgs_.readFromNonRT()->first});
    });

  // Initialize the buffer, so the initial read will also be valid
  auto context = node_->get_node_options().context();

  RCLCPP_INFO(
    get_logger(), "Waiting for first two messages on '%s'", encoder_subscriber_->get_topic_name());

  // FIXME(SuperJappie08): Add configurable timeout
  EncoderMsg first_msg;
  if (!rclcpp::wait_for_message(first_msg, encoder_subscriber_, context)) {
    // FIXME(SuperJappie08): Could be FAILURE as well but depends on what to do with the configuration states
    // TODO(SuperJappie08): Add LOG message
    return hardware_interface::CallbackReturn::ERROR;
  }

  EncoderMsg second_msg;
  if (!rclcpp::wait_for_message(second_msg, encoder_subscriber_, context)) {
    // FIXME(SuperJappie08): Could be FAILURE as well but depends on what to do with the configuration states
    // TODO(SuperJappie08): Add LOG message
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(
    get_logger(), "Recieved the intial two messages on '%s'",
    encoder_subscriber_->get_topic_name());

  latest_msgs_.initRT(
    {std::make_shared<const EncoderMsg>(second_msg),
     std::make_shared<const EncoderMsg>(first_msg)});

  executor_ = rclcpp::executors::SingleThreadedExecutor::make_shared();

  executor_->add_node(node_);

  executor_thread_.reset(new std::thread(std::bind(&rclcpp::Executor::spin, executor_)));

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn EncoderSensor::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  cleanup_node_communication();

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn EncoderSensor::on_shutdown(
  const rclcpp_lifecycle::State & previous_state)
{
  // No action required in UNKNOWN, UNCONFIGURED and FINALIZED
  if (hardware_interface::lifecycleStateThatRequiresNoAction(previous_state.id())) {
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  // In states INACTIVE and ACTIVE the executor is running
  cleanup_node_communication();

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn EncoderSensor::on_error(
  const rclcpp_lifecycle::State & previous_state)
{
  return on_shutdown(previous_state);
}

hardware_interface::return_type EncoderSensor::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  auto newest_msg = latest_msgs_.readFromRT()->first;
  auto older_msg = latest_msgs_.readFromRT()->second;

  if (!newest_msg || !older_msg) {
    RCLCPP_ERROR(get_logger(), "There are no msgs");
    return hardware_interface::return_type::ERROR;
  }

  for (auto joint_state : joint_states_) {
    if (!joint_state->get_interface_name().compare(hardware_interface::HW_IF_POSITION)) {
      double position = ((double)newest_msg->value) / ticks_per_rotation_ * std::numbers::pi * 2.0;

      if (joint_state->set_value(position)) {
        continue;
      }
    } else if (!joint_state->get_interface_name().compare(hardware_interface::HW_IF_VELOCITY)) {
      double difference = ((double)(newest_msg->value - older_msg->value)) / ticks_per_rotation_ *
                          std::numbers::pi * 2.0;
      // FIXME(SuperJappie08): It works with senconds?
      auto dt =
        (rclcpp::Time(newest_msg->header.stamp) - rclcpp::Time(older_msg->header.stamp)).seconds();
      double velocity = difference / dt;

      if (joint_state->set_value(velocity)) {
        continue;
      }
    } else {
      RCLCPP_WARN_ONCE(
        get_logger(), "Unimplemented state_interface: %s", joint_state->get_name().c_str());
      continue;
    }

    RCLCPP_WARN(
      get_logger(), "Setting the value of state interface '%s' failed",
      joint_state->get_name().c_str());
  }

  return hardware_interface::return_type::OK;
}

void EncoderSensor::stop_executor() noexcept
{
  if (executor_ && executor_->is_spinning()) {
    executor_->cancel();
    if (executor_thread_ && executor_thread_->joinable()) {
      executor_thread_->join();
    }
  }
}

void EncoderSensor::cleanup_node_communication()
{
  stop_executor();

  executor_->remove_node(node_);

  // NOTE(SuperJappie08): Cleaning up the executor thread might be uncessairy, however it is good
  //                      practice and ensures the executor itself can be cleaned up.
  executor_thread_.reset();
  executor_.reset();

  latest_msgs_.reset();

  encoder_subscriber_.reset();
  node_.reset();
}

}  // namespace mirte_modular_hardware

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(mirte_modular_hardware::EncoderSensor, hardware_interface::SensorInterface)
