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

ExecutorPosesThroughNavigation::ExecutorPosesThroughNavigation(std::string node_name)
: ExecutorBase(node_name)
{
  nav_through_poses_action_client_ =
    rclcpp_action::create_client<nav2_msgs::action::NavigateThroughPoses>(
    this, "navigate_through_poses");

  // trigger slam
  stop_lidar_trigger_pub_ = create_publisher<std_msgs::msg::Bool>("stop_lidar_relocation", 10);
  stop_vision_trigger_pub_ = create_publisher<std_msgs::msg::Bool>("stop_vision_relocation", 10);

  // trigger navigation
  stop_nav_trigger_sub_ = this->create_subscription<std_msgs::msg::Bool>(
    "stop_nav_trigger",
    rclcpp::SystemDefaultsQoS(),
    std::bind(
      &ExecutorPosesThroughNavigation::HandleTriggerStopCallback, this,
      std::placeholders::_1));

  // localization_lifecycle_ = std::make_shared<LifecycleController>("localization_node");
  map_server_lifecycle_ = std::make_shared<LifecycleController>("map_server");

  // poses though
  nav_client_ = std::make_unique<nav2_lifecycle_manager::LifecycleManagerClient>(
    "lifecycle_manager_navigation");

  // spin
  std::thread{[this]() {
      rclcpp::spin(this->get_node_base_interface());
    }
  }.detach();
}

ExecutorPosesThroughNavigation::~ExecutorPosesThroughNavigation()
{
}

void ExecutorPosesThroughNavigation::Start(const AlgorithmMGR::Goal::ConstSharedPtr goal)
{
  INFO("Poses Through navigation started");
  ReportPreparationStatus();

  // Set vision and lidar flag
  SetLocationType(goal->outdoor);

  // Check all depends is ok
  bool ready = IsDependsReady();
  if (!ready) {
    ERROR("Poses Through navigation lifecycle depend start up failed.");
    ReportPreparationFinished(AlgorithmMGR::Feedback::NAVIGATION_FEEDBACK_NAVIGATING_AB_FAILURE);
    task_abort_callback_();
    return;
  }

  if (velocity_smoother_ == nullptr) {
    velocity_smoother_ = std::make_shared<nav2_util::ServiceClient<MotionServiceCommand>>(
      "velocity_adaptor_gait", shared_from_this());
  }

  // Smoother walk
  VelocitySmoother();

  // Send goal request
  bool success = StartNavThroughPoses(goal->poses);
  if (!success) {
    ERROR("Poses Through send target goal request failed.");
    ReportPreparationFinished(AlgorithmMGR::Feedback::NAVIGATION_FEEDBACK_NAVIGATING_AB_FAILURE);
    task_abort_callback_();
    return;
  }

  // 结束激活进度的上报
  ReportPreparationFinished(AlgorithmMGR::Feedback::NAVIGATION_FEEDBACK_NAVIGATING_AB_SUCCESS);
  INFO("Poses Through send target goal request success.");
}

void ExecutorPosesThroughNavigation::Stop(
  const StopTaskSrv::Request::SharedPtr request,
  StopTaskSrv::Response::SharedPtr response)
{
  (void)request;
  INFO("Poses Through will stop");
  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  if (nav_through_poses_goal_handle_ != nullptr) {
    auto future_cancel = nav_through_poses_action_client_->async_cancel_goal(
      nav_through_poses_goal_handle_);
  } else {
    SetFeedbackCode(AlgorithmMGR::Feedback::NAVIGATION_FEEDBACK_NAVIGATING_AB_FAILURE);
    task_abort_callback_();
    ERROR("Poses Through will stop failed.");
  }

  nav_through_poses_goal_handle_.reset();
  response->result = StopTaskSrv::Response::SUCCESS;

  StopReportPreparationThread();
  SetFeedbackCode(AlgorithmMGR::Feedback::NAVIGATION_FEEDBACK_NAVIGATING_AB_SUCCESS);
  INFO("Poses Through Stoped success");
  task_cancle_callback_();
}

void ExecutorPosesThroughNavigation::Cancel()
{
  INFO("Poses Through Canceled");
}

void ExecutorPosesThroughNavigation::HandleGoalResponseCallback(
  NavThroughPosesGoalHandle::SharedPtr goal_handle)
{
  (void)goal_handle;
  INFO("Poses Through Goal accepted");
}

void ExecutorPosesThroughNavigation::HandleFeedbackCallback(
  NavThroughPosesGoalHandle::SharedPtr,
  const std::shared_ptr<const nav2_msgs::action::NavigateThroughPoses::Feedback> feedback)
{
  (void)feedback;
  // INFO("Navigation feedback, distance_remaining : %f", feedback->distance_remaining);
  SetFeedbackCode(AlgorithmMGR::Feedback::NAVIGATION_FEEDBACK_NAVIGATING_AB_RUNNING);
  task_feedback_callback_(feedback_);
}

void ExecutorPosesThroughNavigation::HandleResultCallback(
  const NavThroughPosesGoalHandle::WrappedResult result)
{
  nav_through_poses_goal_handle_.reset();
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      INFO("Poses Through have arrived target goal success");
      SetFeedbackCode(AlgorithmMGR::Feedback::NAVIGATION_FEEDBACK_NAVIGATING_AB_SUCCESS);
      task_success_callback_();
      break;
    case rclcpp_action::ResultCode::ABORTED:
      ERROR("Poses Through run target goal aborted");
      SetFeedbackCode(AlgorithmMGR::Feedback::NAVIGATION_FEEDBACK_NAVIGATING_AB_FAILURE);
      task_abort_callback_();
      break;
    case rclcpp_action::ResultCode::CANCELED:
      ERROR("Poses Through run target goal canceled");
      SetFeedbackCode(AlgorithmMGR::Feedback::NAVIGATION_FEEDBACK_NAVIGATING_AB_FAILURE);
      task_cancle_callback_();
      break;
    default:
      ERROR("Poses Through run target goal unknown result code");
      SetFeedbackCode(AlgorithmMGR::Feedback::NAVIGATION_FEEDBACK_NAVIGATING_AB_FAILURE);
      task_abort_callback_();
      break;
  }
}

void ExecutorPosesThroughNavigation::HandleTriggerStopCallback(
  const std_msgs::msg::Bool::SharedPtr msg)
{
  INFO("Reset location module and reset some sensor.");
  if (msg == nullptr) {
    return;
  }

  if (msg->data) {
    ReleaseSources();
  }
}

bool ExecutorPosesThroughNavigation::IsDependsReady()
{
  if (nav_client_->is_active() != nav2_lifecycle_manager::SystemStatus::ACTIVE) {
    if (!nav_client_->startup()) {
      WARN("Navigation client lifecycle startup failed.");
      return false;
    }
  }

  // map server lifecycle configure  and deactivate
  if (!map_server_lifecycle_->IsActivate()) {
    bool success = map_server_lifecycle_->Configure();
    if (!success) {
      ERROR("Configure map server lifecycle configure state failed.");
      return false;
    }

    success = map_server_lifecycle_->Startup();
    if (!success) {
      ERROR("Configure map server lifecycle activate state failed.");
      return false;
    }
  }
  return true;
}

bool ExecutorPosesThroughNavigation::IsConnectServer()
{
  auto is_action_server_ready =
    nav_through_poses_action_client_->wait_for_action_server(std::chrono::seconds(5));
  if (!is_action_server_ready) {
    ERROR("Poses Through action server is not available. Is the initial pose set?");
    return false;
  }
  return true;
}

bool ExecutorPosesThroughNavigation::StartNavThroughPoses(
  const std::vector<geometry_msgs::msg::PoseStamped> & poses)
{
  nav_through_poses_goal_.poses = poses;
  Debug2String(poses);

  // Send the goal pose
  auto send_goal_options =
    rclcpp_action::Client<nav2_msgs::action::NavigateThroughPoses>::SendGoalOptions();

  send_goal_options.result_callback =
    std::bind(
    &ExecutorPosesThroughNavigation::HandleResultCallback,
    this, std::placeholders::_1);
  send_goal_options.goal_response_callback =
    std::bind(
    &ExecutorPosesThroughNavigation::HandleGoalResponseCallback,
    this, std::placeholders::_1);
  send_goal_options.feedback_callback =
    std::bind(
    &ExecutorPosesThroughNavigation::HandleFeedbackCallback,
    this, std::placeholders::_1, std::placeholders::_2);

  auto future_goal_handle =
    nav_through_poses_action_client_->async_send_goal(
    nav_through_poses_goal_, send_goal_options);

  std::chrono::seconds timeout{5};
  if (future_goal_handle.wait_for(timeout) != std::future_status::ready) {
    ERROR("Wait navigation server timeout.");
    return false;
  }

  // Get the goal handle and save so that we can check on completion in the timer callback
  nav_through_poses_goal_handle_ = future_goal_handle.get();
  if (!nav_through_poses_goal_handle_) {
    ERROR("Navigation Poses Though Goal was rejected by server");
    return false;
  }
  return true;
}

bool ExecutorPosesThroughNavigation::VelocitySmoother()
{
  while (!velocity_smoother_->wait_for_service(std::chrono::seconds(5s))) {
    if (!rclcpp::ok()) {
      ERROR("Connect velocity adaptor service timeout");
      return false;
    }
  }

  // Set request data
  auto request = std::make_shared<MotionServiceCommand::Request>();
  std::vector<float> step_height{0.01, 0.01};
  request->motion_id = 303;
  request->value = 2;
  request->step_height = step_height;

  // Send request
  auto future_result = velocity_smoother_->invoke(request, std::chrono::seconds(5s));
  return future_result->result;
}

void ExecutorPosesThroughNavigation::Debug2String(
  const std::vector<geometry_msgs::msg::PoseStamped> & poses)
{
  INFO("Robot poses though:");
  for (std::size_t index = 0; index < poses.size(); index++) {
    INFO(
      "\t[pose %lu] [%lf, %lf]",
      index, poses[index].pose.position.x, poses[index].pose.position.y);
  }
}

void ExecutorPosesThroughNavigation::ReleaseSources()
{
  INFO("Release all sources and reset all lifecycle default state.");
  auto command = std::make_shared<std_msgs::msg::Bool>();
  command->data = true;
  stop_vision_trigger_pub_->publish(*command);

  // if (IsUseVisionLocation()) {
  //   stop_vision_trigger_pub_->publish(*command);
  // } else if (IsUseLidarLocation()) {
  //   stop_lidar_trigger_pub_->publish(*command);
  // }
  ResetDefaultValue();
}

bool ExecutorPosesThroughNavigation::IsUseVisionLocation()
{
  return use_vision_slam_;
}

bool ExecutorPosesThroughNavigation::IsUseLidarLocation()
{
  return use_lidar_slam_;
}

void ExecutorPosesThroughNavigation::SetLocationType(bool outdoor)
{
  if (outdoor) {
    INFO("Current location mode use vision slam.");
    use_vision_slam_ = true;
  } else {
    INFO("Current location mode use lidar slam.");
    use_lidar_slam_ = true;
  }
}

void ExecutorPosesThroughNavigation::ResetDefaultValue()
{
  use_vision_slam_ = false;
  use_lidar_slam_ = false;
}

}  // namespace algorithm
}  // namespace cyberdog
