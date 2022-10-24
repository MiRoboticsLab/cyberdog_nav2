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
  callback_group_ =
    action_client_node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  client_vision_algo_ =
    action_client_node_->create_client<protocol::srv::AlgoManager>("algo_manager");

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
    action_client_node_, "tracking_target");
  target_tracking_goal_ = mcr_msgs::action::TargetTracking::Goal();
  std::thread{[this] {this->executor_->spin();}}.detach();
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
  INFO("VisionTracking Stop");
  OnCancel();
  response->result = StopTaskSrv::Response::SUCCESS;
}

void ExecutorVisionTracking::Cancel()
{
  INFO("VisionTracking Cancel");
  OnCancel();
}

void ExecutorVisionTracking::OnCancel()
{
  if (start_vision_tracking_) {
    if (target_tracking_goal_handle_ != nullptr) {
      INFO("Cancel target_tracking_goal_handle_");
      auto future_cancel =
        target_tracking_action_client_->async_cancel_goal(target_tracking_goal_handle_);
    } else {
      WARN("target_tracking_goal_handle_ is nullptr");
      if (!DeactivateDepsLifecycleNodes(50000)) {
        ERROR("DeactivateDepsLifecycleNodes failed");
      }
    }
    StopReportPreparationThread();
    task_cancle_callback_();
    INFO("OnCancel completed");
    start_vision_tracking_ = false;
    target_tracking_goal_handle_.reset();
    return;
  }
}

// TODO(PDF):
bool ExecutorVisionTracking::CallVisionTrackAlgo()
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
      return false;
    }
    INFO("service not available, waiting again...");
  }
  INFO("client_vision_algo_ service available! async_send_request");
  auto result = client_vision_algo_->async_send_request(request);
  if (result.wait_for(std::chrono::milliseconds(5000)) == std::future_status::timeout) {
    ERROR("Cannot Get result client_vision_algo_");
    return false;
  } else {
    INFO("Get result, enable result: %d", result.get()->result_enable);
    return true;
  }
}
// TODO(PDF):
uint8_t ExecutorVisionTracking::StartVisionTracking(uint8_t relative_pos, float keep_distance)
{
  (void)relative_pos;
  (void)keep_distance;
  SetFeedbackCode(500);
  INFO("FeedbackCode: %d", feedback_->feedback_code);
  if (!ActivateDepsLifecycleNodes(this->get_name(), 50000)) {
    ERROR("ActivateDepsLifecycleNodes failed");
    ReportPreparationFinished(AlgorithmMGR::Feedback::TASK_PREPARATION_FAILED);
    if (!DeactivateDepsLifecycleNodes(50000)) {
      ERROR("DeactivateDepsLifecycleNodes failed");
    }
    task_abort_callback_();
    return Navigation::Result::NAVIGATION_RESULT_TYPE_FAILED;
  }
  if (!CallVisionTrackAlgo()) {
    ERROR("CallVisionTrackAlgo failed");
    ReportPreparationFinished(AlgorithmMGR::Feedback::TASK_PREPARATION_FAILED);
    if (!DeactivateDepsLifecycleNodes(50000)) {
      ERROR("DeactivateDepsLifecycleNodes failed");
    }
    task_abort_callback_();
    return Navigation::Result::NAVIGATION_RESULT_TYPE_FAILED;
  }
  start_vision_tracking_ = true;
  SetFeedbackCode(501);
  INFO("FeedbackCode: %d", feedback_->feedback_code);
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
    if (!DeactivateDepsLifecycleNodes(50000)) {
      ERROR("DeactivateDepsLifecycleNodes failed");
    }
    task_abort_callback_();
    res->success = false;
    return;
  }
  SetFeedbackCode(502);
  INFO("FeedbackCode: %d", feedback_->feedback_code);
  auto is_action_server_ready =
    target_tracking_action_client_->wait_for_action_server(
    std::chrono::seconds(5));
  if (!is_action_server_ready) {
    ERROR("Tracking target action server is not available.");
    ReportPreparationFinished(AlgorithmMGR::Feedback::TASK_PREPARATION_FAILED);
    if (!DeactivateDepsLifecycleNodes(50000)) {
      ERROR("DeactivateDepsLifecycleNodes failed");
    }
    res->success = false;
    task_abort_callback_();
    return;
  }
  INFO("is_action_server_ready success");
  // Send the goal pose
  target_tracking_goal_.relative_pos = 1;
  target_tracking_goal_.keep_distance = 1.1;
  // Enable result awareness by providing an empty lambda function
  auto send_goal_options = rclcpp_action::Client<
    mcr_msgs::action::TargetTracking>::SendGoalOptions();
  send_goal_options.goal_response_callback =
    std::bind(
    &ExecutorVisionTracking::HandleGoalResponseCallback,
    this, std::placeholders::_1);
  send_goal_options.feedback_callback =
    std::bind(
    &ExecutorVisionTracking::HandleFeedbackCallback,
    this, std::placeholders::_1, std::placeholders::_2);
  send_goal_options.result_callback =
    std::bind(
    &ExecutorVisionTracking::HandleResultCallback,
    this, std::placeholders::_1);
  auto future_goal_handle = target_tracking_action_client_->async_send_goal(
    target_tracking_goal_, send_goal_options);
  INFO("target_tracking_action_client_ async_send_goal");
  if (future_goal_handle.wait_for(std::chrono::milliseconds(5000)) == std::future_status::timeout) {
    ERROR("Cannot Get result target_tracking_action_client_");
    ReportPreparationFinished(AlgorithmMGR::Feedback::TASK_PREPARATION_FAILED);
    if (!DeactivateDepsLifecycleNodes(50000)) {
      ERROR("DeactivateDepsLifecycleNodes async_send_goal failed");
    }
    task_abort_callback_();
    return;
  } else {
    INFO("target_tracking_action_client_  success");
  }
  // Get the goal handle and save so that we can check on completion in the
  // timer callback
  target_tracking_goal_handle_ = future_goal_handle.get();
  if (!target_tracking_goal_handle_) {
    ERROR("Goal was rejected by server");
    ReportPreparationFinished(AlgorithmMGR::Feedback::TASK_PREPARATION_FAILED);
    if (!DeactivateDepsLifecycleNodes(50000)) {
      ERROR("DeactivateDepsLifecycleNodes failed");
    }
    task_abort_callback_();
    return;
  }
}
void ExecutorVisionTracking::HandleFeedbackCallback(
  TargetTrackingGoalHandle::SharedPtr,
  const std::shared_ptr<const McrTargetTracking::Feedback> feedback)
{
  // INFO("HandleFeedbackCallback: %d", feedback->exception_code);
  switch (feedback->exception_code) {
    case 0:
      feedback_->feedback_code = 503;
      // AlgorithmMGR::Feedback::NAVIGATION_FEEDBACK_BASE_TRACKING_NOEXCEPTION;
      break;

    case 1000:
      feedback_->feedback_code = 504;
      // AlgorithmMGR::Feedback::NAVIGATION_FEEDBACK_BASE_TRACKING_DETECOTOREXCEPTION;
      break;

    case 2000:
      feedback_->feedback_code =
        AlgorithmMGR::Feedback::NAVIGATION_FEEDBACK_BASE_TRACKING_TFEXCEPTION;
      break;

    case 3000:
      feedback_->feedback_code =
        AlgorithmMGR::Feedback::NAVIGATION_FEEDBACK_BASE_TRACKING_PLANNNEREXCEPTION;
      break;

    case 4000:
      feedback_->feedback_code =
        AlgorithmMGR::Feedback::NAVIGATION_FEEDBACK_BASE_TRACKING_CONTROLLEREXCEPTION;
      break;

    default:
      break;
  }
  INFO("FeedbackCode: %d", feedback_->feedback_code);
  // task_feedback_callback_(feedback_);
}

void ExecutorVisionTracking::HandleResultCallback(
  const TargetTrackingGoalHandle::WrappedResult result)
{
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      INFO("Vision Tracking reported succeeded");
      // if (!OperateDepsNav2LifecycleNodes(this->get_name(), Nav2LifecycleMode::kPause)) {
      //   ERROR("OperateDepsNav2LifecycleNodes failed.");
      // }
      // if (!DeactivateDepsLifecycleNodes(timeout=50000)) {
      //   ERROR("DeactivateDepsLifecycleNodes failed");
      // }
      break;
    case rclcpp_action::ResultCode::ABORTED:
      ERROR("Vision Tracking reported aborted");
      // if (!OperateDepsNav2LifecycleNodes(this->get_name(), Nav2LifecycleMode::kPause)) {
      //   ERROR("OperateDepsNav2LifecycleNodes failed.");
      // }
      // if (!DeactivateDepsLifecycleNodes()) {
      //   ERROR("DeactivateDepsLifecycleNodes failed");
      // }
      // StopReportPreparationThread();
      // target_tracking_goal_handle_.reset();
      // task_abort_callback_();
      // if (!DeactivateDepsLifecycleNodes(timeout=50000)) {
      //   ERROR("DeactivateDepsLifecycleNodes failed");
      // }
      break;
    case rclcpp_action::ResultCode::CANCELED:
      WARN("Vision Tracking reported canceled");
      // if (!OperateDepsNav2LifecycleNodes(this->get_name(), Nav2LifecycleMode::kPause)) {
      //   ERROR("OperateDepsNav2LifecycleNodes failed.");
      // }
      if (!DeactivateDepsLifecycleNodes(50000)) {
        ERROR("DeactivateDepsLifecycleNodes failed");
      }
      // StopReportPreparationThread();
      // target_tracking_goal_handle_.reset();
      // task_cancle_callback_();
      break;
    default:
      ERROR("Vision Tracking reported unknown result code");
      // if (!OperateDepsNav2LifecycleNodes(this->get_name(), Nav2LifecycleMode::kPause)) {
      //   ERROR("OperateDepsNav2LifecycleNodes failed.");
      // }
      // if (!DeactivateDepsLifecycleNodes()) {
      //   ERROR("DeactivateDepsLifecycleNodes failed");
      // }
      // StopReportPreparationThread();
      // target_tracking_goal_handle_.reset();
      // task_abort_callback_();
      // if (!DeactivateDepsLifecycleNodes(timeout=50000)) {
      //   ERROR("DeactivateDepsLifecycleNodes failed");
      // }
      break;
  }
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

  // auto client_cb = [timeout](rclcpp::Client<protocol::srv::BodyRegion>::SharedFuture future) {
  //     std::future_status status = future.wait_for(timeout);

  //     if (status == std::future_status::ready) {
  //       if (0 != future.get()->success) {
  //         return false;
  //       } else {
  //         return true;
  //       }
  //     } else {
  //       return false;
  //     }
  //   };

  // auto result = client->async_send_request(req, client_cb);
  auto result = client->async_send_request(req);
  if (result.wait_for(std::chrono::milliseconds(5000)) == std::future_status::timeout) {
    ERROR("Cannot Get result TrackingClientCallService");
    return false;
  } else {
    INFO("result.get()->success: %d", result.get()->success);
    return true;
  }
  // return true;
}
}  // namespace algorithm
}  // namespace cyberdog
