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

#ifndef MIRTE_MODULAR_HARDWARE__ENCODER_SENSOR_HPP_
#define MIRTE_MODULAR_HARDWARE__ENCODER_SENSOR_HPP_

#include <chrono>
#include <limits>
#include <memory>

/* FIXME(SuperJappie08): TO SEPERATE INCLUDES */
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/sensor_interface.hpp>
#include <realtime_tools/realtime_buffer.hpp>
/* FIXME(SuperJappie08): TO SEPERATE INCLUDES */
#include <rclcpp/duration.hpp>
#include <rclcpp/macros.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp_lifecycle/state.hpp>
/* FIXME(SuperJappie08): TO SEPERATE INCLUDES */

#include "mirte_modular_hardware/hardware_interface_helper.hpp"
#include "mirte_modular_hardware/visibility_control.hpp"
#include "mirte_msgs/msg/encoder.hpp"

#if !HARDWARE_INTERFACE_NODE_AVAILABLE
#include <rclcpp/executor.hpp>

#include "mirte_modular_hardware/helpers.hpp"
#endif

namespace mirte_modular_hardware
{

// NOTE(SuperJappie08): Maybe rename to JointEncoderSensor
class EncoderSensor : public hardware_interface::SensorInterface
{
private:
  using EncoderMsg = mirte_msgs::msg::Encoder;

public:
  RCLCPP_SHARED_PTR_DEFINITIONS(EncoderSensor)

  MIRTE_MODULAR_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_init(const HARDWARE_INTERFACE_INIT_PARAM & params) override;

  MIRTE_MODULAR_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  MIRTE_MODULAR_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  MIRTE_MODULAR_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_error(
    const rclcpp_lifecycle::State & previous_state) override;

  MIRTE_MODULAR_HARDWARE_PUBLIC
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // Parameters
  // NOTE(SuperJappie08): Maybe convert this to ticks/rad (or rad/ticks) whatever makes sense
  double ticks_per_rotation_ = std::numeric_limits<double>::quiet_NaN();
  std::chrono::milliseconds initial_message_timeout_{500};
  bool updates_only_ = false;
  rclcpp::Time last_used_msg_time_;

  realtime_tools::RealtimeBuffer<std::pair<EncoderMsg::ConstSharedPtr, EncoderMsg::ConstSharedPtr>>
    latest_msgs_{{nullptr, nullptr}};
  rclcpp::Subscription<EncoderMsg>::SharedPtr encoder_subscriber_ = nullptr;

#if !HARDWARE_INTERFACE_NODE_AVAILABLE

private:
  /* NOTE(SuperJappie08): https://github.com/husarion/rosbot_hardware_interfaces/blob/main/src/rosbot_system.cpp
   and other use multithreaded, test this and non shared? */
  // Defaults to SingleThreadedExecutor
  std::unique_ptr<ExecutorThread> executor_thread_ = nullptr;
  rclcpp::Node::SharedPtr node_ = nullptr;
  rclcpp::Node::SharedPtr get_node() const { return node_; }
  rclcpp::Executor::SharedPtr get_executor() const { return executor_thread_->get_executor(); }

#endif
};
}  // namespace mirte_modular_hardware

#endif  // MIRTE_MODULAR_HARDWARE__ENCODER_SENSOR_HPP_
