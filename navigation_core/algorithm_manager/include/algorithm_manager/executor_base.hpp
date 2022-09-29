// Copyright (c) 2021 Beijing Xiaomi Mobile Software Co., Ltd. All rights reserved.
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

#ifndef ALGORITHM_MANAGER__EXECUTOR_BASE_HPP_
#define ALGORITHM_MANAGER__EXECUTOR_BASE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "protocol/action/navigation.hpp"
// #include "algorithm_manager/algorithm_task_manager.hpp"
#include "cyberdog_common/cyberdog_log.hpp"
namespace cyberdog
{
namespace algorithm
{

class ExecutorBase : public rclcpp::Node
{

public:
  enum class ExecutorStatus : uint8_t
  {
    kIdle = 0,
    kWorking = 1,
  };

public:
  ExecutorBase(std::string node_name) : rclcpp::Node(node_name)
  {
  }
  ~ExecutorBase(){};
  virtual void Start() = 0; 
  virtual void Stop() = 0; 
  virtual void Cancel() = 0; 
  virtual void GetFeedback(protocol::action::Navigation::Feedback::SharedPtr feedback) = 0;
  virtual ExecutorStatus GetStatus() { return ExecutorStatus::kIdle; }
protected:


private:



};   // class ExecutorBase
}  // namespace algorithm
}  // namespace cyberdog
#endif  // ALGORITHM_MANAGER__EXECUTOR_BASE_HPP_
