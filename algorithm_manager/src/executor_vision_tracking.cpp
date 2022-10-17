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
{
  auto options = rclcpp::NodeOptions().arguments(
    {"--ros-args", "-r", std::string("__node:=") + get_name() + "_client", "--"});
  action_client_node_ = std::make_shared<rclcpp::Node>("_", options);
  executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  executor_->add_node(action_client_node_);
  callback_group_ = action_client_node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  rclcpp::SubscriptionOptions option;
  option.callback_group = callback_group_;

  // TODO(PDF):
  // client_realsense_manager_ =
  //   std::make_shared<nav2_util::LifecycleServiceClient>("camera/camera");
  // client_vision_manager_ =
  //   std::make_shared<nav2_util::LifecycleServiceClient>("vision_manager");
  // client_tracking_manager_ =
  //   std::make_shared<nav2_util::LifecycleServiceClient>("tracking");
  client_vision_algo_ =
    action_client_node_->create_client<protocol::srv::AlgoManager>("algo_manager");
  // Create service server
  service_tracking_object_ = action_client_node_->create_service<BodyRegionT>(
    "tracking_object_srv", std::bind(
      &ExecutorVisionTracking::TrackingSrvCallback, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3),
      rmw_qos_profile_services_default,
      callback_group_
  );
  client_tracking_object_ = action_client_node_->create_client<BodyRegionT>(
    "tracking_object",
    rmw_qos_profile_services_default,
    callback_group_
  );
  target_tracking_action_client_ =
    rclcpp_action::create_client<mcr_msgs::action::TargetTracking>(
    action_client_node_, "tracking_target",callback_group_);

  target_tracking_goal_ = mcr_msgs::action::TargetTracking::Goal();

  std::thread{[this]{this->executor_->spin();}}.detach();
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
}

void ExecutorVisionTracking::Stop(
  const StopTaskSrv::Request::SharedPtr request,
  StopTaskSrv::Response::SharedPtr response)
{
  (void)request;
  INFO("Vision Tracking Stopped");
  if (start_vision_tracking_) {
    if (target_tracking_goal_handle_) {
      auto future_cancel =
        target_tracking_action_client_->async_cancel_goal(target_tracking_goal_handle_);
    }
    StopReportPreparationThread();
    OperateDepsNav2LifecycleNodes(this->get_name(), Nav2LifecycleMode::kPause);
    DeactivateDepsLifecycleNodes();
    target_tracking_goal_handle_.reset();
    response->result = StopTaskSrv::Response::SUCCESS :
    start_vision_tracking_ = false;
    return;
  }

}

void ExecutorVisionTracking::Cancel()
{
  INFO("Vision Tracking Cancel");
  if (start_vision_tracking_) {
    if (target_tracking_goal_handle_) {
      auto future_cancel =
        target_tracking_action_client_->async_cancel_goal(target_tracking_goal_handle_);
    }
    StopReportPreparationThread();
    OperateDepsNav2LifecycleNodes(this->get_name(), Nav2LifecycleMode::kPause);
    DeactivateDepsLifecycleNodes();
    target_tracking_goal_handle_.reset();
    start_vision_tracking_ = false;
    return;
  }
}

bool ExecutorVisionTracking::ActivateDepsLifecycleNodes()
{
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
        return false;
      }
      INFO("Success to active %s", client.name.c_str());
    }
  }
  return true;
}

bool ExecutorVisionTracking::DeactivateDepsLifecycleNodes()
{
  for (auto client : GetDepsLifecycleNodes(this->get_name())) {
    if (client.lifecycle_client->get_state() ==
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
    {
      INFO("Lifecycle node %s already be inactive", client.name.c_str());
      continue;
    } else {
      if (!client.lifecycle_client->change_state(
          lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE))
      {
        ERROR("Get error when deactive %s", client.name.c_str());
      }
      INFO("Success to deactive %s", client.name.c_str());
    }
  }
  return true;
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
  INFO("client_vision_algo_ service available! async_send_request");
  auto result = client_vision_algo_->async_send_request(request);
  if (result.wait_for(std::chrono::milliseconds(10000)) == std::future_status::timeout) {
    ERROR("Cannot Get result client_vision_algo_");
  } else {
    INFO("enable result: %d", result.get()->result_enable);
  }
}

// TODO(PDF):
uint8_t ExecutorVisionTracking::StartVisionTracking(uint8_t relative_pos, float keep_distance)
{
  (void)relative_pos;
  (void)keep_distance;
  SetFeedbackCode(500);
  if (!ActivateDepsLifecycleNodes()) {
    ReportPreparationFinished(AlgorithmMGR::Feedback::TASK_PREPARATION_FAILED);
    DeactivateDepsLifecycleNodes();
    task_abort_callback_();
    return Navigation::Result::NAVIGATION_RESULT_TYPE_FAILED;
  }
  INFO("CallVisionTrackAlgo");
  start_vision_tracking_ = true;
  CallVisionTrackAlgo();
  SetFeedbackCode(501);
  INFO("SetFeedbackCode(501)");
  return Navigation::Result::NAVIGATION_RESULT_TYPE_ACCEPT;
}
// TODO(PDF):
void ExecutorVisionTracking::TrackingSrvCallback(
  const std::shared_ptr<rmw_request_id_t>,
  const std::shared_ptr<BodyRegionT::Request> req,
  std::shared_ptr<BodyRegionT::Response> res)
{
  // send tracking_object to cyberdog_vision
  if (TrackingClientCallService(client_tracking_object_, req->roi)) {
    INFO("TrackingClientCallService success");
    res->success = true;
  } else {
    ERROR("TrackingClientCallService failed");
    ReportPreparationFinished(AlgorithmMGR::Feedback::TASK_PREPARATION_FAILED);
    task_abort_callback_();
    res->success = false;
    return;
  }
  SetFeedbackCode(502);
  if (!OperateDepsNav2LifecycleNodes(this->get_name(), Nav2LifecycleMode::kStartUp)) {
    ReportPreparationFinished(AlgorithmMGR::Feedback::TASK_PREPARATION_FAILED);
    ERROR("OperateDepsNav2LifecycleNodes not available.");
    task_abort_callback_();
    return;
  }
  INFO("OperateDepsNav2LifecycleNodes success");
  auto is_action_server_ready =
    target_tracking_action_client_->wait_for_action_server(
    std::chrono::seconds(5));
  if (!is_action_server_ready) {
    // client_nav_.pause();
    ERROR("Tracking target action server is not available.");
    ReportPreparationFinished(AlgorithmMGR::Feedback::TASK_PREPARATION_FAILED);
    task_abort_callback_();
    OperateDepsNav2LifecycleNodes(this->get_name(), Nav2LifecycleMode::kPause);
    res->success = false;
    return;
  }
  INFO("is_action_server_ready success");
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
  INFO("async_send_goal");
  auto future_goal_handle = target_tracking_action_client_->async_send_goal(
    target_tracking_goal_, send_goal_options);
  if (future_goal_handle.wait_for(std::chrono::milliseconds(20000)) != std::future_status::ready) {
    ERROR("Cannot Get result target_tracking_action_client_");
    ReportPreparationFinished(AlgorithmMGR::Feedback::TASK_PREPARATION_FAILED);
    task_abort_callback_();
    // client_nav_.pause();
    OperateDepsNav2LifecycleNodes(this->get_name(), Nav2LifecycleMode::kPause);
    return;
  } else {
    INFO("enable result: %d", future_goal_handle.get());
  }
  // Get the goal handle and save so that we can check on completion in the
  // timer callback
  target_tracking_goal_handle_ = future_goal_handle.get();
  if (!target_tracking_goal_handle_) {
    ERROR("Goal was rejected by server");
    ReportPreparationFinished(AlgorithmMGR::Feedback::TASK_PREPARATION_FAILED);
    task_abort_callback_();
    // client_nav_.pause();
    OperateDepsNav2LifecycleNodes(this->get_name(), Nav2LifecycleMode::kPause);
    return;
  }
  // vision_action_client_feedback_ = 503;
  SetFeedbackCode(503);
}
// TODO(PDF):
bool ExecutorVisionTracking::TrackingClientCallService(
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
