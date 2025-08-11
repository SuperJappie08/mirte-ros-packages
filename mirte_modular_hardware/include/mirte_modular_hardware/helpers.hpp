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

#ifndef MIRTE_MODULAR_HARDWARE__HELPERS_HPP_
#define MIRTE_MODULAR_HARDWARE__HELPERS_HPP_

#include <memory>
#include <thread>
/* FIXME(SuperJappie08): TO SEPERATE INCLUDES */
#include <rclcpp/executor.hpp>
#include <rclcpp/executor_options.hpp>
#include <rclcpp/macros.hpp>

#include "mirte_modular_hardware/visibility_control.hpp"

namespace mirte_modular_hardware
{
struct ThreadJoiner
{
  MIRTE_MODULAR_HARDWARE_LOCAL
  ThreadJoiner(){};
  void operator()(std::thread * ptr) const noexcept;
};

class ExecutorThread
{
public:
  RCLCPP_DISABLE_COPY(ExecutorThread)
  RCLCPP_SMART_PTR_DEFINITIONS_NOT_COPYABLE(ExecutorThread)

  MIRTE_MODULAR_HARDWARE_LOCAL
  ExecutorThread(const rclcpp::ExecutorOptions & options = rclcpp::ExecutorOptions()) noexcept;

  MIRTE_MODULAR_HARDWARE_LOCAL
  ExecutorThread(rclcpp::Executor::SharedPtr executor) noexcept;

  MIRTE_MODULAR_HARDWARE_LOCAL
  ~ExecutorThread() noexcept;

  MIRTE_MODULAR_HARDWARE_LOCAL
  rclcpp::Executor::SharedPtr get_executor() const noexcept { return executor_; }

private:
  rclcpp::Executor::SharedPtr executor_;
  std::unique_ptr<std::thread> thread_;
};
}  // namespace mirte_modular_hardware

#endif  // MIRTE_MODULAR_HARDWARE__HELPERS_HPP_
