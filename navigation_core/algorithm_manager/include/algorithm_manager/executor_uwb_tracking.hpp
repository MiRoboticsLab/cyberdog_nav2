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

#ifndef ALGORITHM_MANAGER__EXECUTOR_UWB_TRACKING_HPP_
#define ALGORITHM_MANAGER__EXECUTOR_UWB_TRACKING_HPP_

#include <memory>
#include <string>
#include "algorithm_manager/executor_base.hpp"

namespace cyberdog
{
namespace algorithm
{

class ExecutorUwbTracking : public ExecutorBase
{
public:
  explicit ExecutorUwbTracking(std::string node_name);
  void Start(const AlgorithmMGR::Goal::ConstSharedPtr goal) override;
  void Stop(const StopTaskSrv::Request::SharedPtr request) override;
  void Cancel() override;

private:
  void HandleGoalResponseCallback(TargetTrackingGoalHandle::SharedPtr goal_handle)
  {
    (void)goal_handle;
    INFO("Goal accepted");
  }
  void HandleFeedbackCallback(
    TargetTrackingGoalHandle::SharedPtr,
    const std::shared_ptr<const McrTargetTracking::Feedback> feedback);
  void HandleResultCallback(const TargetTrackingGoalHandle::WrappedResult goal_handle);
  void StatusFeedbackMonitor();
  ExecutorData executor_uwb_tracking_data_;
  rclcpp_action::Client<mcr_msgs::action::TargetTracking>::SharedPtr
    target_tracking_action_client_;
  mcr_msgs::action::TargetTracking::Goal target_tracking_goal_;
  TargetTrackingGoalHandle::SharedPtr target_tracking_goal_handle_;
};  // class ExecutorUwbTracking
}  // namespace algorithm
}  // namespace cyberdog
#endif  // ALGORITHM_MANAGER__EXECUTOR_UWB_TRACKING_HPP_
