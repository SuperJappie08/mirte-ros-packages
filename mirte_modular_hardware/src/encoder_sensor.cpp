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
#include <rclcpp/rate.hpp>
#include <rclcpp/timer.hpp>
#include <rclcpp/wait_for_message.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include "mirte_modular_hardware/encoder_sensor.hpp"
#include "mirte_msgs/msg/encoder.hpp"

namespace mirte_modular_hardware
{
hardware_interface::CallbackReturn EncoderSensor::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SensorInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (auto ticks_per_rotation_raw = info_.hardware_parameters.find("ticks_per_rotation");
      ticks_per_rotation_raw != info_.hardware_parameters.end()) {
    ticks_per_rotation_ = hardware_interface::stod(ticks_per_rotation_raw->second);
    RCLCPP_INFO(get_logger(), "Loaded 'ticks_per_rotation' [%f]", ticks_per_rotation_);
  } else {
    RCLCPP_FATAL(
      get_logger(),
      "Missing required 'ticks_per_rotation' hardware parameter, to indicate the number of encoder "
      "ticks corresponding to a full rotation.");
    return hardware_interface::CallbackReturn::ERROR;
  }

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

  if (info_.joints.size() != 1) {
    RCLCPP_FATAL(
      get_logger(), "Exactly 1 Joint is expected on the '%s' interface type, but %zu were defined.",
      info_.hardware_plugin_name.c_str(), info_.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  // TODO(SuperJappie08): Process Joint

  // TODO(SuperJappie08): Process general Parameters

  // TODO(SuperJappie08): Setup initialize encoder communication

  // TODO: Check configuration of interfaces in urdf
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn EncoderSensor::on_configure(
  const rclcpp_lifecycle::State & previous_state)
{
  executor_ = rclcpp::executors::SingleThreadedExecutor::make_shared();

  auto node_options =
    rclcpp::NodeOptions().start_parameter_event_publisher(false).start_parameter_services(false);
  node_ =
    rclcpp::Node::make_shared("mirte_modular_hardware_encoder_sensor_" + get_name(), node_options);

  // FIXME(SuperJappie08): Make configurable
  auto topic_name = "topic";

  // TODO(SuperJappie08): Investigate if a single message buffer (1 msg) could be used if the previous position state is used to calculate the speed.
  //                    - Pros: Less confusing and coping, out-zeroing when crashed
  //                    - Cons: If message arrive late it zeros the velocity, could introduce chatter
  encoder_subscriber_ = node_->create_subscription<mirte_msgs::msg::Encoder>(
    // FIXME(SuperJappie08): Figure out if keep_last(5) (default) or keep_last(1/2/3) is better
    topic_name, rclcpp::SensorDataQoS() /* .keep_last(1)*/,
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

  executor_->add_node(node_);

  executor_thread_ = std::make_unique<std::thread>(std::bind(&rclcpp::Executor::spin, executor_));

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn EncoderSensor::on_activate(
  const rclcpp_lifecycle::State & previous_state)
{
  // TODO(SuperJappie08): IMPLEMENT?
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn EncoderSensor::on_shutdown(
  const rclcpp_lifecycle::State & previous_state)
{
  // TODO(SuperJappie08): IMPLEMENT?
  if (executor_ && executor_->is_spinning()) {
    executor_->cancel();
    if (executor_thread_ && executor_thread_->joinable()) {
      executor_thread_->join();
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn EncoderSensor::on_error(
  const rclcpp_lifecycle::State & previous_state)
{
  // FIXME(SuperJappie08): Temporarily Check if executor is still spinning;
  if (executor_ && executor_->is_spinning()) {
    executor_->cancel();
    if (executor_thread_ && executor_thread_->joinable()) {
      executor_thread_->join();
    }
  }

  // TODO(SuperJappie08): Error handling (Temporary Failure)
  return hardware_interface::CallbackReturn::FAILURE;
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
      double velocity = difference / dt;  // / 1e9; // * 1.0e-9;

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
}  // namespace mirte_modular_hardware

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(mirte_modular_hardware::EncoderSensor, hardware_interface::SensorInterface)
