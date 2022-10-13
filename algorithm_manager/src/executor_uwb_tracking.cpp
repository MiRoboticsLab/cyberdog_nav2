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
    action_client_node_, "tracking_target");
  std::thread{[this]() {rclcpp::spin(action_client_node_);}}.detach();
}

void ExecutorUwbTracking::Start(const AlgorithmMGR::Goal::ConstSharedPtr goal)
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
      task_abort_callback_();
      return;
  }

  INFO("UWB Tracking will start");
  // 在激活依赖节点前需要开始上报激活进度
  ReportPreparationStatus();
  if (!OperateDepsNav2LifecycleNodes(this->get_name(), Nav2LifecycleMode::kStartUp)) {
    ReportPreparationFinished(AlgorithmMGR::Feedback::TASK_PREPARATION_FAILED);
    task_abort_callback_();
    return;
  }

  for (auto client : GetDepsLifecycleNodes(this->get_name())) {
    if (client.lifecycle_client->get_state() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
      INFO("Lifecycle node %s already be active", client.name.c_str());
      continue;
    } else {
      if (!client.lifecycle_client->change_state(
          lifecycle_msgs::msg::Transition::
          TRANSITION_CONFIGURE))
      {
        WARN("Get error when configuring %s, try to active", client.name.c_str());
      }
      if (!client.lifecycle_client->change_state(
          lifecycle_msgs::msg::Transition::
          TRANSITION_ACTIVATE))
      {
        ERROR("Get error when activing %s", client.name.c_str());
        ReportPreparationFinished(AlgorithmMGR::Feedback::TASK_PREPARATION_FAILED);
        task_abort_callback_();
        return;
      }
      INFO("Success to active %s", client.name.c_str());
    }
  }

  // TODO(Harvey): 当有Realsense的依赖时
  // TODO(Harvey): 当有其他没有继承Nav2Lifecycle的依赖节点时
  auto is_action_server_ready =
    target_tracking_action_client_->wait_for_action_server(
    std::chrono::seconds(5));
  if (!is_action_server_ready) {
    ERROR("TrackingTarget action server is not available.");
    OperateDepsNav2LifecycleNodes(this->get_name(), Nav2LifecycleMode::kPause);
    ReportPreparationFinished(AlgorithmMGR::Feedback::TASK_PREPARATION_FAILED);
    task_abort_callback_();
    return;
  }
  // 结束激活进度的上报
  ReportPreparationFinished(AlgorithmMGR::Feedback::TASK_PREPARATION_SUCCESS);
  // Send the goal pose
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
  auto future_goal_handle = target_tracking_action_client_->async_send_goal(
    target_tracking_goal, send_goal_options);
  if (future_goal_handle.wait_for(server_timeout_) != std::future_status::ready) {
    ERROR("Send TrackingTarget goal failed");
    OperateDepsNav2LifecycleNodes(this->get_name(), Nav2LifecycleMode::kPause);
    task_abort_callback_();
    return;
  }
  target_tracking_goal_handle_ = future_goal_handle.get();
  if (!target_tracking_goal_handle_) {
    ERROR("TrackingTarget Goal was rejected by server");
    OperateDepsNav2LifecycleNodes(this->get_name(), Nav2LifecycleMode::kPause);
    task_abort_callback_();
    return;
  }
  INFO("UWB Tracking started");
}

void ExecutorUwbTracking::Stop(
  const StopTaskSrv::Request::SharedPtr request,
  StopTaskSrv::Response::SharedPtr response)
{
  (void)request;
  INFO("UWB Tracking will stop");
  if (target_tracking_goal_handle_ != nullptr) {
    // 只有在向底层执行器发送目标后才需要发送取消指令
    target_tracking_action_client_->async_cancel_goal(target_tracking_goal_handle_);
  } else {
    task_abort_callback_();
  }
  StopReportPreparationThread();
  target_tracking_goal_handle_.reset();
  response->result = OperateDepsNav2LifecycleNodes(this->get_name(), Nav2LifecycleMode::kPause) ?
    StopTaskSrv::Response::SUCCESS :
    StopTaskSrv::Response::FAILED;
  INFO("UWB Tracking Stoped");
}

void ExecutorUwbTracking::Cancel()
{
  INFO("UWB Tracking will cancel");
  if (target_tracking_goal_handle_ != nullptr) {
    target_tracking_action_client_->async_cancel_goal(target_tracking_goal_handle_);
  } else {
    task_abort_callback_();
  }
  StopReportPreparationThread();
  OperateDepsNav2LifecycleNodes(this->get_name(), Nav2LifecycleMode::kPause);
  target_tracking_goal_handle_.reset();
  INFO("UWB Tracking Canceled");
}

void ExecutorUwbTracking::HandleFeedbackCallback(
  TargetTrackingGoalHandle::SharedPtr,
  const std::shared_ptr<const McrTargetTracking::Feedback> feedback)
{
  feedback_->feedback_code = feedback->exception_code;
  task_feedback_callback_(feedback_);
}

void ExecutorUwbTracking::HandleResultCallback(const TargetTrackingGoalHandle::WrappedResult result)
{
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      INFO("UWB Tracking reported succeeded");
      task_success_callback_();
      break;
    case rclcpp_action::ResultCode::ABORTED:
      ERROR("UWB Tracking reported aborted");
      task_abort_callback_();
      break;
    case rclcpp_action::ResultCode::CANCELED:
      ERROR("UWB Tracking reported canceled");
      task_cancle_callback_();
      break;
    default:
      ERROR("UWB Tracking reported unknown result code");
      task_abort_callback_();
      break;
  }
}
}  // namespace algorithm
}  // namespace cyberdog
