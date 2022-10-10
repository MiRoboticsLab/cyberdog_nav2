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
#include "algorithm_manager/executor_vision_tracking.hpp"

namespace cyberdog
{
namespace algorithm
{

ExecutorVisionTracking::ExecutorVisionTracking(std::string node_name)
: ExecutorBase(node_name)
{}

void ExecutorVisionTracking::Start(const AlgorithmMGR::Goal::ConstSharedPtr goal)
{
  (void)goal;
  INFO("Vision Tracking started");
}

void ExecutorVisionTracking::Stop(
  const StopTaskSrv::Request::SharedPtr request,
  StopTaskSrv::Response::SharedPtr response)
{
  (void)request;
  (void)response;
  INFO("Vision Tracking Stopped");
}

void ExecutorVisionTracking::Cancel()
{
  INFO("Vision Tracking Stopped");
}

}  // namespace algorithm
}  // namespace cyberdog
