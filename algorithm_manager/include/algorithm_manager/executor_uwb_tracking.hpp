// Copyright (c) 2023 Beijing Xiaomi Mobile Software Co., Ltd. All rights reserved.
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
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace cyberdog
{
namespace algorithm
{

class ExecutorUwbTracking : public ExecutorBase
{
public:
  explicit ExecutorUwbTracking(std::string node_name);
  void Start(const AlgorithmMGR::Goal::ConstSharedPtr goal) override;
  void Stop(
    const StopTaskSrv::Request::SharedPtr request,
    StopTaskSrv::Response::SharedPtr response) override;
  void Cancel() override;

private:
  void HandleGoalResponseCallback(TargetTrackingGoalHandle::SharedPtr goal_handle);
  void HandleFeedbackCallback(
    TargetTrackingGoalHandle::SharedPtr,
    const std::shared_ptr<const McrTargetTracking::Feedback> feedback);
  void HandleResultCallback(const TargetTrackingGoalHandle::WrappedResult goal_handle);
  void OnCancel(StopTaskSrv::Response::SharedPtr response = nullptr);
  void UpdateBehaviorStatus(const BehaviorManager::BehaviorStatus & status);
  void ResetAllDeps();
  void ResetLifecycles();
  bool StartupRealsenseData(bool enable);
  void QueryVisualObstacleAvoidanceStatusCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void SwitchVisualObstacleAvoidanceCallback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response);
  ExecutorData executor_uwb_tracking_data_;
  rclcpp_action::Client<mcr_msgs::action::TargetTracking>::SharedPtr
    target_tracking_action_client_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr stair_monitor_client_;
  mcr_msgs::action::TargetTracking::Goal target_tracking_goal_;
  TargetTrackingGoalHandle::SharedPtr target_tracking_goal_handle_;
  rclcpp::executors::MultiThreadedExecutor::SharedPtr executor_;
  std::mutex target_tracking_server_mutex_;
  std::condition_variable target_tracking_server_cv_;
  std::shared_ptr<BehaviorManager> behavior_manager_;
  BehaviorManager::BehaviorStatus behavior_status_;
  std::mutex task_mutex_;
  bool cancel_tracking_result_{true};
  bool stair_detect_{false}, static_detect_{false};
  bool deps_lifecycle_activated_ {false};
  bool enable_visual_obstacle_avoidance_{true};
  std::shared_ptr<nav2_util::ServiceClient<std_srvs::srv::SetBool>> realsense_client_{nullptr};
  rclcpp::CallbackGroup::SharedPtr callbackgroup_query_visual_obstacle_avoidance_status_ {nullptr};
  rclcpp::CallbackGroup::SharedPtr callbackgroup_switch_visual_obstacle_avoidance_ {nullptr};
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr query_visual_avoidance_status_service_ {nullptr};  // NOLINT
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr switch_visual_avoidance_service_ {nullptr};
};  // class ExecutorUwbTracking
}  // namespace algorithm
}  // namespace cyberdog
#endif  // ALGORITHM_MANAGER__EXECUTOR_UWB_TRACKING_HPP_
