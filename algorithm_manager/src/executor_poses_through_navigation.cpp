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

#include <memory>
#include <vector>
#include <string>

#include "algorithm_manager/executor_poses_through_navigation.hpp"

namespace cyberdog
{
namespace algorithm
{

ExecutorPosesThoughNavigation::ExecutorPosesThoughNavigation(std::string node_name)
: ExecutorBase(node_name)
{
  // spin
  std::thread{[this]() {
      rclcpp::spin(this->get_node_base_interface());
    }
  }.detach();
}

ExecutorPosesThoughNavigation::~ExecutorPosesThoughNavigation()
{
}

void ExecutorPosesThoughNavigation::Start(const AlgorithmMGR::Goal::ConstSharedPtr goal)
{
}

void ExecutorPosesThoughNavigation::Stop(
  const StopTaskSrv::Request::SharedPtr request,
  StopTaskSrv::Response::SharedPtr response)
{
}

void ExecutorPosesThoughNavigation::Cancel()
{
}

}  // namespace algorithm
}  // namespace cyberdog
