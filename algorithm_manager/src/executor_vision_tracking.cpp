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
: ExecutorBase(node_name),client_nav_("lifecycle_manager_navigation")
{
  auto options = rclcpp::NodeOptions().arguments(
    {"--ros-args --remap __node:=vision_tracking_target_action_client"});
  action_client_node_ = std::make_shared<rclcpp::Node>("_", options);

  // TODO(PDF):
  client_realsense_manager_ =
    std::make_shared<nav2_util::LifecycleServiceClient>("camera/camera");
  client_vision_manager_ =
    std::make_shared<nav2_util::LifecycleServiceClient>("vision_manager");
  client_tracking_manager_ =
    std::make_shared<nav2_util::LifecycleServiceClient>("tracking");
  client_vision_algo_ =
    action_client_node_->create_client<protocol::srv::AlgoManager>("algo_manager");
  // Create service server
  service_tracking_object_ = action_client_node_->create_service<BodyRegionT>(
    "tracking_object_srv", std::bind(
      &ExecutorVisionTracking::TrackingSrv_callback, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  client_tracking_object_ = action_client_node_->create_client<BodyRegionT>("tracking_object");
  target_tracking_action_client_ =
    rclcpp_action::create_client<mcr_msgs::action::TargetTracking>(
    action_client_node_, "tracking_target");

  target_tracking_goal_ = mcr_msgs::action::TargetTracking::Goal();

  std::thread{[this]() {rclcpp::spin(action_client_node_);}}.detach();
}

void ExecutorVisionTracking::Start(const AlgorithmMGR::Goal::ConstSharedPtr goal)
{
  INFO("Vision Tracking starting");
  ReportPreparationStatus();
  uint8_t goal_result = StartVisionTracking(goal->relative_pos, goal->keep_distance);
  if (goal_result != Navigation::Result::NAVIGATION_RESULT_TYPE_ACCEPT) {
    ERROR("ExecutorVisionTracking::Start Error");
    ReportPreparationFinished(AlgorithmMGR::Feedback::TASK_PREPARATION_FAILED);
    task_abort_callback_();
  }
  return;
}

void ExecutorVisionTracking::Stop(
  const StopTaskSrv::Request::SharedPtr request,
  StopTaskSrv::Response::SharedPtr response)
{
  (void)request;
  (void)response;
  INFO("Vision Tracking Stopped");
  Cancel();
  return;
}

void ExecutorVisionTracking::Cancel()
{
  INFO("Vision Tracking Cancel");
  OnCancel();
  return;
}
// TODO(PDF):
void ExecutorVisionTracking::OnCancel()
{
  if (start_vision_tracking_) {
    if ((!client_realsense_manager_->change_state(
        lifecycle_msgs::msg::Transition::
        TRANSITION_DEACTIVATE)))
    {
      ERROR("realsense_manager lifecycle TRANSITION_DEACTIVATE failed");
    }
    if ((!client_vision_manager_->change_state(
        lifecycle_msgs::msg::Transition::
        TRANSITION_DEACTIVATE)))
    {
      ERROR("vision_manager lifecycle TRANSITION_DEACTIVATE failed");
    }
    if (target_tracking_goal_handle_) {
      if (!client_tracking_manager_->change_state(
          lifecycle_msgs::msg::Transition::
          TRANSITION_DEACTIVATE))
      {
        ERROR("tracking_manager_ lifecycle TRANSITION_DEACTIVATE failed");
      }
      auto future_cancel =
        target_tracking_action_client_->async_cancel_goal(target_tracking_goal_handle_);

      if (rclcpp::spin_until_future_complete(
          client_node_, future_cancel,
          server_timeout_) !=
        rclcpp::FutureReturnCode::SUCCESS)
      {
        ERROR("Failed to cancel goal");
      } else {
        // target_tracking_action_client_.reset();
        INFO("canceled navigation goal");
      }
      client_nav_.pause();
    }
    start_vision_tracking_ = false;
    return;
  }
}
// TODO(PDF):
void ExecutorVisionTracking::FeedbackMonitor()
{
  // RCLCPP_INFO(client_node_->get_logger(), "Navigation status monitor ...");
  if (start_vision_tracking_) {
    rclcpp::spin_some(client_node_);
    {
      feedback_->feedback_code = vision_action_client_feedback_;
      task_feedback_callback_(feedback_);
      // state_machine_.postEvent(new ROSActionQEvent(QActionState::ACTIVE));
    }
    return;
  }
}

// TODO(PDF):
void ExecutorVisionTracking::CallVisionTrackAlgo()
{
  auto request = std::make_shared<protocol::srv::AlgoManager::Request>();
  protocol::msg::AlgoList algo;
  // algo.algo_module = protocol::msg::AlgoList::ALGO_FACE;
  // request->algo_enable.push_back(algo);
  algo.algo_module = protocol::msg::AlgoList::ALGO_BODY;
  request->algo_enable.push_back(algo);
  algo.algo_module = protocol::msg::AlgoList::ALGO_REID;
  request->algo_enable.push_back(algo);
  // algo.algo_module = protocol::msg::AlgoList::ALGO_GESTURE;
  // request->algo_enable.push_back(algo);
  // algo.algo_module = protocol::msg::AlgoList::ALGO_KEYPOINTS;
  // request->algo_enable.push_back(algo);
  // algo.algo_module = protocol::msg::AlgoList::ALGO_FOCUS;
  // request->algo_enable.push_back(algo);
  request->open_age = true;
  request->open_emotion = true;

  while (!client_vision_algo_->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      ERROR("Interrupted while waiting for the service. Exiting.");
      return;
    }
    INFO("service not available, waiting again...");
  }

  auto result = client_vision_algo_->async_send_request(request);
  // Wait for the result.
  if (rclcpp::spin_until_future_complete(client_node_, result) ==
    rclcpp::FutureReturnCode::SUCCESS)
  {
    INFO("enable result: %d", result.get()->result_enable);
  } else {
    ERROR("Failed to call service");
  }
}

// TODO(PDF):
uint8_t ExecutorVisionTracking::StartVisionTracking(uint8_t relative_pos, float keep_distance)
{
  (void)relative_pos;
  (void)keep_distance;
  // vision_action_client_feedback_ = 500;
  SetFeedbackCode(500);
  // feedback_timer_ = this->create_wall_timer(
  //   2000ms, std::bind(&ExecutorVisionTracking::FeedbackMonitor, this));
  // start realsense lifecycle node
  if (client_realsense_manager_->get_state() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    if ((!client_realsense_manager_->change_state(
        lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE)))
    {
      ERROR("realsense_manager lifecycle TRANSITION_CONFIGURE failed");
    }

    if (!client_realsense_manager_->change_state(
        lifecycle_msgs::msg::Transition::
        TRANSITION_ACTIVATE))
    {
      ERROR("realsense_manager lifecycle TRANSITION_ACTIVATE failed");
      return Navigation::Result::NAVIGATION_RESULT_TYPE_FAILED;
    }
    INFO("realsense_manager  TRANSITION_ACTIVATE success");
  }
  // start vision_manager lifecycle node
  if (client_vision_manager_->get_state() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    if ((!client_vision_manager_->change_state(
        lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE)))
    {
      ERROR("vision_manager lifecycle TRANSITION_CONFIGURE failed");
    }

    if (!client_vision_manager_->change_state(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE))
    {
      ERROR("vision_manager lifecycle TRANSITION_ACTIVATE failed");
      return Navigation::Result::NAVIGATION_RESULT_TYPE_FAILED;
    }
    INFO("vision_manager lifecycle TRANSITION_ACTIVATE success");
  }
  start_vision_tracking_ = true;
  CallVisionTrackAlgo();
  SetFeedbackCode(501);
  // vision_action_client_feedback_ = 501;
  return Navigation::Result::NAVIGATION_RESULT_TYPE_ACCEPT;
}
// TODO(PDF):
void ExecutorVisionTracking::TrackingSrv_callback(
  const std::shared_ptr<rmw_request_id_t>,
  const std::shared_ptr<BodyRegionT::Request> req,
  std::shared_ptr<BodyRegionT::Response> res)
{
  // send tracking_object to cyberdog_vision
  if (TrackingClient_call_service(client_tracking_object_, req->roi)) {
    INFO("TrackingClient_call_service success");
    res->success = true;
  } else {
    ERROR("TrackingClient_call_service failed");
    ReportPreparationFinished(AlgorithmMGR::Feedback::TASK_PREPARATION_FAILED);
    task_abort_callback_();
    res->success = false;
    return;
  }
  // start tracking_manager lifecycle node
  if (client_tracking_manager_->get_state() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    if (!client_tracking_manager_->change_state(
        lifecycle_msgs::msg::Transition::
        TRANSITION_CONFIGURE))
    {
      ERROR("tracking_manager_ lifecycle TRANSITION_CONFIGURE failed");
    }
    if (!client_tracking_manager_->change_state(
        lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE))
    {
      ERROR("tracking_manager_ lifecycle TRANSITION_ACTIVATE failed");
      ReportPreparationFinished(AlgorithmMGR::Feedback::TASK_PREPARATION_FAILED);
      task_abort_callback_();
      res->success = false;
      return;
    }
  }
  SetFeedbackCode(502);
  // vision_action_client_feedback_ = 502;
  // return;
  // start navigation stack
  if (client_nav_.is_active() != nav2_lifecycle_manager::SystemStatus::ACTIVE) {
    if (!client_nav_.startup()) {
      ERROR("start navigation stack");
      ReportPreparationFinished(AlgorithmMGR::Feedback::TASK_PREPARATION_FAILED);
      task_abort_callback_();
      res->success = false;
      return;
    }
  }
  auto is_action_server_ready =
    target_tracking_action_client_->wait_for_action_server(
    std::chrono::seconds(5));
  if (!is_action_server_ready) {
    client_nav_.pause();
    ERROR("Tracking target action server is not available.");
    ReportPreparationFinished(AlgorithmMGR::Feedback::TASK_PREPARATION_FAILED);
    task_abort_callback_();
    res->success = false;
    return;
  }
  // Send the goal pose
  // navigation_goal_.pose = pose;
  target_tracking_goal_.relative_pos = 1;
  // INFO("NavigateToPose will be called using the BT Navigator's default behavior tree.");

  // Enable result awareness by providing an empty lambda function
  auto send_goal_options = rclcpp_action::Client<
    mcr_msgs::action::TargetTracking>::SendGoalOptions();
  send_goal_options.result_callback = [this](auto) {
      INFO("Tracking target send_goal callback");
      // SenResult();
      // target_tracking_goal_handle_.reset();
    };

  auto future_goal_handle = target_tracking_action_client_->async_send_goal(
    target_tracking_goal_, send_goal_options);
  if (rclcpp::spin_until_future_complete(
      client_node_, future_goal_handle,
      server_timeout_) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    ERROR("Send goal call failed");
    ReportPreparationFinished(AlgorithmMGR::Feedback::TASK_PREPARATION_FAILED);
    task_abort_callback_();
    client_nav_.pause();
    return;
  }

  // Get the goal handle and save so that we can check on completion in the
  // timer callback
  target_tracking_goal_handle_ = future_goal_handle.get();
  if (!target_tracking_goal_handle_) {
    ERROR("Goal was rejected by server");
    ReportPreparationFinished(AlgorithmMGR::Feedback::TASK_PREPARATION_FAILED);
    task_abort_callback_();
    client_nav_.pause();
    return;
  }
  // vision_action_client_feedback_ = 503;
  SetFeedbackCode(503);
}
// TODO(PDF):
bool ExecutorVisionTracking::TrackingClient_call_service(
  rclcpp::Client<protocol::srv::BodyRegion>::SharedPtr & client,
  const sensor_msgs::msg::RegionOfInterest & roi)
{
  auto req = std::make_shared<protocol::srv::BodyRegion::Request>();
  req->roi = roi;

  std::chrono::nanoseconds timeout = std::chrono::nanoseconds(-1);
  while (!client->wait_for_service(timeout)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
      return false;
    }
    RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
  }

  auto client_cb = [timeout](rclcpp::Client<protocol::srv::BodyRegion>::SharedFuture future) {
      std::future_status status = future.wait_for(timeout);

      if (status == std::future_status::ready) {
        if (0 != future.get()->success) {
          return false;
        } else {
          return true;
        }
      } else {
        return false;
      }
    };

  auto result = client->async_send_request(req, client_cb);
  return true;
}
}  // namespace algorithm
}  // namespace cyberdog
