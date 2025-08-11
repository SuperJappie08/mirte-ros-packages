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

#include "mirte_modular_hardware/helpers.hpp"

#include <functional>
#include <memory>
#include <thread>
/* FIXME(SuperJappie08): TO SEPERATE INCLUDES */
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rcpputils/asserts.hpp>

namespace mirte_modular_hardware
{

void ThreadJoiner::operator()(std::thread * ptr) const noexcept
{
  if (ptr != nullptr) {
    if (ptr->joinable()) {
      ptr->join();
    }
  }
}

ExecutorThread::ExecutorThread(const rclcpp::ExecutorOptions & options) noexcept
: ExecutorThread(rclcpp::executors::SingleThreadedExecutor::make_shared(options))
{
}

ExecutorThread::ExecutorThread(rclcpp::Executor::SharedPtr executor) noexcept : executor_(executor)
{
  rcpputils::require_true(
    !executor_->is_spinning(), "The provided executor must not be spinning yet!");

  thread_ = std::make_unique<std::thread>(std::bind(&rclcpp::Executor::spin, executor_));
}

ExecutorThread::~ExecutorThread() noexcept
{
  if (executor_ && executor_->is_spinning()) {
    executor_->cancel();
  }

  if (thread_ && thread_->joinable()) {
    thread_->join();
  }
}

}  // namespace mirte_modular_hardware
