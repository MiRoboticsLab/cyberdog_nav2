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
#include "algorithm_manager/executor_uwb_tracking.hpp"

namespace cyberdog
{
namespace algorithm
{

ExecutorUwbTracking::ExecutorUwbTracking(std::string node_name)
: ExecutorBase(node_name)
{
  auto options = rclcpp::NodeOptions().arguments(
    {"--ros-args --remap __node:=tracking_target_action_client"});
  action_client_node_ = std::make_shared<rclcpp::Node>("_", options);
  target_tracking_action_client_ =
    rclcpp_action::create_client<mcr_msgs::action::TargetTracking>(
    action_client_node_, "tracking_target_fake");
  std::thread{[this]() {rclcpp::spin(action_client_node_);}}.detach();
}

bool ExecutorUwbTracking::Start(const AlgorithmMGR::Goal::ConstSharedPtr goal)
{
  mcr_msgs::action::TargetTracking_Goal target_tracking_goal;
  target_tracking_goal.keep_distance = goal->keep_distance;
  switch (goal->relative_pos) {
    case AlgorithmMGR::Goal::TRACING_AUTO:
      target_tracking_goal.relative_pos = McrTargetTracking::Goal::AUTO;
      break;

    case AlgorithmMGR::Goal::TRACING_LEFT:
      target_tracking_goal.relative_pos = McrTargetTracking::Goal::LEFT;
      break;

    case AlgorithmMGR::Goal::TRACING_RIGHT:
      target_tracking_goal.relative_pos = McrTargetTracking::Goal::RIGHT;
      break;

    case AlgorithmMGR::Goal::TRACING_BEHIND:
      target_tracking_goal.relative_pos = McrTargetTracking::Goal::BEHIND;
      break;

    default:
      ERROR("Get Invalid tracking pos");
      return false;
  }

  INFO("UWB Tracking will start");
  ReportPreparationStatus();
  if (!LaunchNav2LifeCycleNode(GetNav2LifecycleMgrClient(LifecycleClientID::kNav)) ||
    !LaunchNav2LifeCycleNode(GetNav2LifecycleMgrClient(LifecycleClientID::kMcrUwb)))
  {
    ERROR("Failed to Launch lifecycle nodes");
    ReportPreparationFinished(AlgorithmMGR::Feedback::TASK_PREPARATION_FAILED);
    return false;
  }
  auto is_action_server_ready =
    target_tracking_action_client_->wait_for_action_server(
    std::chrono::seconds(5));
  if (!is_action_server_ready) {
    ERROR("TrackingTarget action server is not available.");
    GetNav2LifecycleMgrClient(LifecycleClientID::kNav)->pause();
    GetNav2LifecycleMgrClient(LifecycleClientID::kMcrUwb)->pause();
    ReportPreparationFinished(AlgorithmMGR::Feedback::TASK_PREPARATION_FAILED);
    return false;
  }

  // Send the goal pose
  // Enable result awareness by providing an empty lambda function
  auto send_goal_options =
    rclcpp_action::Client<McrTargetTracking>::SendGoalOptions();
  send_goal_options.goal_response_callback =
    std::bind(
    &ExecutorUwbTracking::HandleGoalResponseCallback,
    this, std::placeholders::_1);
  send_goal_options.feedback_callback =
    std::bind(
    &ExecutorUwbTracking::HandleFeedbackCallback,
    this, std::placeholders::_1, std::placeholders::_2);
  send_goal_options.result_callback =
    std::bind(
    &ExecutorUwbTracking::HandleResultCallback,
    this, std::placeholders::_1);
  ReportPreparationFinished(AlgorithmMGR::Feedback::TASK_PREPARATION_SUCCESS);
  auto future_goal_handle = target_tracking_action_client_->async_send_goal(
    target_tracking_goal, send_goal_options);
  if (future_goal_handle.wait_for(server_timeout_) != std::future_status::ready) {
    ERROR("Send TrackingTarget goal failed");
    GetNav2LifecycleMgrClient(LifecycleClientID::kNav)->pause();
    GetNav2LifecycleMgrClient(LifecycleClientID::kMcrUwb)->pause();
    return false;
  }
  target_tracking_goal_handle_ = future_goal_handle.get();
  if (!target_tracking_goal_handle_) {
    ERROR("TrackingTarget Goal was rejected by server");
    GetNav2LifecycleMgrClient(LifecycleClientID::kNav)->pause();
    GetNav2LifecycleMgrClient(LifecycleClientID::kMcrUwb)->pause();
    return false;
  }
  INFO("UWB Tracking started");
  return true;
}

void ExecutorUwbTracking::Stop()
{
  INFO("UWB Tracking will stop");
  if (target_tracking_goal_handle_ != nullptr) {
    target_tracking_action_client_->async_cancel_goal(target_tracking_goal_handle_);
  }
  StopReportPreparationThread();
  executor_uwb_tracking_data_.status = ExecutorStatus::kSuccess;
  UpdateExecutorData(executor_uwb_tracking_data_);
  INFO("UWB Tracking Stoped");
}


void ExecutorUwbTracking::Cancel()
{
  INFO("UWB Tracking will cancel");
  if (target_tracking_goal_handle_ != nullptr) {
    target_tracking_action_client_->async_cancel_goal(target_tracking_goal_handle_);
  }
  StopReportPreparationThread();
  executor_uwb_tracking_data_.status = ExecutorStatus::kCanceled;
  UpdateExecutorData(executor_uwb_tracking_data_);
  INFO("UWB Tracking Canceled");
}

void ExecutorUwbTracking::HandleFeedbackCallback(
  TargetTrackingGoalHandle::SharedPtr,
  const std::shared_ptr<const McrTargetTracking::Feedback> feedback)
{
  // INFO("Got feedback: %d", feedback->exception_code);
  executor_uwb_tracking_data_.status = ExecutorStatus::kExecuting;
  executor_uwb_tracking_data_.feedback.feedback_code = feedback->exception_code;
  UpdateExecutorData(executor_uwb_tracking_data_);
}

void ExecutorUwbTracking::HandleResultCallback(const TargetTrackingGoalHandle::WrappedResult result)
{
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      executor_uwb_tracking_data_.status = ExecutorStatus::kSuccess;
      INFO("UWB Tracking reported succeeded");
      break;
    case rclcpp_action::ResultCode::ABORTED:
      executor_uwb_tracking_data_.status = ExecutorStatus::kAborted;
      ERROR("UWB Tracking reported aborted");
      break;
    case rclcpp_action::ResultCode::CANCELED:
      executor_uwb_tracking_data_.status = ExecutorStatus::kAborted;
      ERROR("UWB Tracking reported canceled");
      break;
    default:
      executor_uwb_tracking_data_.status = ExecutorStatus::kAborted;
      ERROR("UWB Tracking reported unknown result code");
      break;
  }
  UpdateExecutorData(executor_uwb_tracking_data_);
}
}  // namespace algorithm
}  // namespace cyberdog