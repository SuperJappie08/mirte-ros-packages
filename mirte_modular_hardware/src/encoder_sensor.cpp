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
#include <cstdint>
#include <memory>
#include <numbers>
/* FIXME(SuperJappie08): TO SEPERATE INCLUDES */
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/lexical_casts.hpp>
#include <hardware_interface/lifecycle_helpers.hpp>
#include <hardware_interface/sensor_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <hardware_interface/types/lifecycle_state_names.hpp>
/* FIXME(SuperJappie08): TO SEPERATE INCLUDES */
#include <rclcpp/callback_group.hpp>
#include <rclcpp/context.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/subscription_options.hpp>
#include <rclcpp/wait_for_message.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include "mirte_modular_hardware/encoder_sensor.hpp"

#if !HARDWARE_INTERFACE_NODE_AVAILABLE
#include <rclcpp/node_options.hpp>

#include "mirte_modular_hardware/helpers.hpp"
#endif

namespace
{
constexpr const auto kTopicKey = "topic";
constexpr const auto kTicksPerRotationKey = "ticks_per_rotation";
constexpr const auto kInitMsgTimeOutKey = "initial_message_timeout_ms";
constexpr const auto kUpdates = "update";

#if !HARDWARE_INTERFACE_NODE_AVAILABLE
constexpr const auto kNodeNamePrefix = "mirte_modular_hardware_encoder_sensor_";
#endif
}  // namespace

namespace mirte_modular_hardware
{

hardware_interface::CallbackReturn EncoderSensor::on_init(
  const HARDWARE_INTERFACE_INIT_PARAM & params)
{
  if (
    hardware_interface::SensorInterface::on_init(params) !=
    hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  const auto hardware_plugin_name = get_hardware_info().hardware_plugin_name;

  // Retrieve general parameters
  if (get_hardware_info().hardware_parameters.contains(kTicksPerRotationKey)) {
    // TODO(SuperJappie08): Possible store this as rad per tick
    ticks_per_rotation_ =
      hardware_interface::stod(get_hardware_info().hardware_parameters.at(kTicksPerRotationKey));
    RCLCPP_INFO(get_logger(), "Loaded '%s' [%f]", kTicksPerRotationKey, ticks_per_rotation_);
  } else {
    RCLCPP_FATAL(
      get_logger(),
      "Missing required '%s' hardware parameter, "
      "to indicate the number of encoder ticks corresponding to a full rotation.",
      kTicksPerRotationKey);
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (get_hardware_info().hardware_parameters.contains(kTopicKey)) {
    RCLCPP_INFO(
      get_logger(), "Using '%s' as the topic name (relative to the hardware node).",
      get_hardware_info().hardware_parameters.at(kTopicKey).c_str());
  } else {
    RCLCPP_FATAL(
      get_logger(),
      "Missing required '%s' hardware parameter, "
      "to indicate the topic (relative to the hardware node).",
      kTopicKey);
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Optional hardware parameters
  if (get_hardware_info().hardware_parameters.contains(kInitMsgTimeOutKey)) {
    RCLCPP_INFO(
      get_logger(),
      "Attempting to parse '%s' parameter [int(in milliseconds)]. (use -1 to disable timeout)",
      kInitMsgTimeOutKey);
    initial_message_timeout_ = std::chrono::milliseconds(
      std::stol(get_hardware_info().hardware_parameters.at(kInitMsgTimeOutKey)));
  }

  if (initial_message_timeout_ == std::chrono::milliseconds(-1)) {
    RCLCPP_WARN(
      get_logger(),
      "The initial message timeout has been disabled, controller could wait indefinitely. "
      "[Use hardware parameter '%s' to configure the timeout]",
      kInitMsgTimeOutKey);
  } else {
    RCLCPP_INFO_STREAM(
      get_logger(), "Using initial message timeout of " << initial_message_timeout_ << ".");
  }

  if (get_hardware_info().hardware_parameters.contains(kUpdates)) {
    updates_only_ =
      hardware_interface::parse_bool(get_hardware_info().hardware_parameters.at(kUpdates));
    RCLCPP_INFO_EXPRESSION(get_logger(), updates_only_, "Using the updates topic.");
    RCLCPP_INFO_EXPRESSION(get_logger(), !updates_only_, "Using the normal topic.");
  } else {
    RCLCPP_INFO(
      get_logger(),
      "Using the normal encoder topic. [use param 'update' to toggle using the updates]");
  }

  // Validate if the configuration is valid.

  if (!get_hardware_info().transmissions.empty()) {
    RCLCPP_FATAL(
      get_logger(),
      "Transmissions are not supported on the '%s' interface type, but they were defined.",
      hardware_plugin_name.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (!get_hardware_info().sensors.empty()) {
    RCLCPP_FATAL(
      get_logger(),
      "Sensor components are not supported on the '%s' interface type, but they were defined.",
      hardware_plugin_name.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (!get_hardware_info().gpios.empty()) {
    RCLCPP_FATAL(
      get_logger(),
      "GPIO components are not supported on the '%s' interface type, but they were defined.",
      hardware_plugin_name.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  // NOTE(SuperJappie08): Theoretically there could also be a interface which listens to multiple encoders/devices.
  //                      However, that makes configuration less clear and the code more complex.
  if (get_hardware_info().joints.size() != 1) {
    RCLCPP_FATAL(
      get_logger(), "Exactly 1 Joint is expected on the '%s' interface type, but %zu were defined.",
      hardware_plugin_name.c_str(), get_hardware_info().joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  auto joint = get_hardware_info().joints[0];

  // Check if the specified joint is consistent with the capabilities of this
  // hardware interface.
  if (!joint.command_interfaces.empty()) {
    RCLCPP_FATAL(
      get_logger(), "The '%s' interface type does not support any command interfaces.",
      hardware_plugin_name.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (joint.is_mimic == hardware_interface::MimicAttribute::TRUE) {
    // TODO(SuperJappie08): Figure out if supporting mimic joints make sense?
    RCLCPP_FATAL(
      get_logger(), "Mimic joints are currently not supported on '%s' interface types.",
      hardware_plugin_name.c_str());
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
        joint.name.c_str(), get_name().c_str(), hardware_plugin_name.c_str());
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
        joint.name.c_str(), get_name().c_str(), hardware_plugin_name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
  } else {
    RCLCPP_FATAL(
      get_logger(),
      "Joint '%s' of hardware interface '%s' [%s] has a unexpected amount of state interfaces. "
      "Expected 2 ['position', 'velocity'], but found %zu interfaces.",
      joint.name.c_str(), get_name().c_str(), hardware_plugin_name.c_str(),
      joint.state_interfaces.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  // TODO(SuperJappie08): possibly verify Joint parameters

  // TODO(SuperJappie08): Process general Parameters

#if !HARDWARE_INTERFACE_NODE_AVAILABLE
  auto node_options =
    rclcpp::NodeOptions().start_parameter_event_publisher(false).start_parameter_services(false);
  node_ = rclcpp::Node::make_shared(kNodeNamePrefix + get_name(), node_options);
  executor_thread_ = std::make_unique<ExecutorThread>();

  get_executor()->add_node(get_node());
#endif

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn EncoderSensor::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // FIXME: MAYBE MOVE THIS TO INIT
  if (!get_node()) {
    RCLCPP_FATAL(
      get_logger(), "Node has not been started for '%s' [%s]", get_name().c_str(),
      get_hardware_info().hardware_plugin_name.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  const auto topic_name = get_hardware_info().hardware_parameters.at(kTopicKey);

  // Initialize the buffer, so the initial read will also be valid

  RCLCPP_INFO(get_logger(), "Waiting for first two messages on '%s'", topic_name.c_str());

  // FIXME(SuperJappie08): Figure out if keep_last(5) (default) or keep_last(1/2/3) is better
  const auto qos = rclcpp::SensorDataQoS() /* .keep_last(1)*/;

  EncoderMsg first_msg;
  EncoderMsg second_msg;
  {
    // NOTE(SuperJappie08): Need to make a temporary callback group and subscriber.
    //                      Since Node already part of an Executor.
    auto context = get_node()->get_node_options().context();
    auto callback_group =
      get_node()->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
    auto sub_options = rclcpp::SubscriptionOptions();
    sub_options.callback_group = callback_group;

    auto subscriber = get_node()->create_subscription<EncoderMsg>(
      topic_name, qos, [](const EncoderMsg::ConstSharedPtr) { return; }, sub_options);

    if (!rclcpp::wait_for_message(first_msg, subscriber, context, initial_message_timeout_)) {
      /* NOTE(SuperJappie08): Return failure since then we would revert to
       unconfigured (if started from a non configured state). If the hardware
       interface fails when start requires a state ros2_control crashes
       (https://github.com/ros-controls/ros2_control/issues/2290) */
      RCLCPP_ERROR_STREAM(
        get_logger(), "Timed out waiting for first Encoder message on '"
                        << topic_name << "'. [Timeout = " << initial_message_timeout_ << "]");
      return hardware_interface::CallbackReturn::FAILURE;
    }

    if (!rclcpp::wait_for_message(second_msg, subscriber, context, initial_message_timeout_)) {
      RCLCPP_ERROR_STREAM(
        get_logger(), "Timed out waiting for second Encoder message on '"
                        << topic_name << "'. [Timeout = " << initial_message_timeout_
                        << "] (hint: Check the frequency of '" << topic_name << "')");
      return hardware_interface::CallbackReturn::FAILURE;
    }
  }

  RCLCPP_INFO(get_logger(), "Recieved the intial two messages on '%s'", topic_name.c_str());

  latest_msgs_.initRT(
    {std::make_shared<const EncoderMsg>(second_msg),
     std::make_shared<const EncoderMsg>(first_msg)});

  std::string final_topic_name = topic_name;
  if (this->updates_only_) {
    final_topic_name += "/update";
  }

  // NOTE(SuperJappie08): Using 2 messages allows to get something closer to
  //                      the instantaneous speed of the wheel
  encoder_subscriber_ = get_node()->create_subscription<EncoderMsg>(
    final_topic_name, qos, [this](const EncoderMsg::ConstSharedPtr msg) {
      this->latest_msgs_.writeFromNonRT({msg, this->latest_msgs_.readFromNonRT()->first});
    });

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn EncoderSensor::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  encoder_subscriber_.reset();

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn EncoderSensor::on_error(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  encoder_subscriber_.reset();

  return hardware_interface::CallbackReturn::SUCCESS;
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
    if (joint_state->get_interface_name() == hardware_interface::HW_IF_POSITION) {
      double position = ((double)newest_msg->value) / ticks_per_rotation_ * std::numbers::pi * 2.0;

      if (joint_state->set_value(position)) [[likely]] {
        continue;
      }
    } else if (joint_state->get_interface_name() == hardware_interface::HW_IF_VELOCITY) {
      int32_t new_ticks = newest_msg->value;
      int32_t old_ticks = older_msg->value;
      int32_t tick_difference = new_ticks - old_ticks;

      double difference = ((double)tick_difference) / ticks_per_rotation_ * std::numbers::pi * 2.0;
      auto dt =
        (rclcpp::Time(newest_msg->header.stamp) - rclcpp::Time(older_msg->header.stamp)).seconds();

      RCLCPP_WARN_EXPRESSION(
        get_logger(), dt <= 0.0,
        "The time difference between the encoder steps is %fs, "
        "check if its source is setup correctly.",
        dt);

      double velocity = difference / dt;

      if (joint_state->set_value(velocity)) [[likely]] {
        continue;
      }
    } else [[unlikely]] {
      RCLCPP_WARN_ONCE(
        get_logger(), "Unimplemented state_interface: %s", joint_state->get_name().c_str());
      return hardware_interface::return_type::ERROR;
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
