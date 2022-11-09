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
  auto sub1_opt = rclcpp::SubscriptionOptions();
  sub1_opt.callback_group = callback_group_;
  // Subscription vision_manager object tracking status
  rclcpp::SensorDataQoS pub_qos;
  pub_qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
  vision_manager_status_sub_ = action_client_node_->create_subscription<TrackingStatusT>(
    "processing_status",
    pub_qos,
    std::bind(
      &ExecutorVisionTracking::VisionManagerStatusCallback, this,
      std::placeholders::_1), sub1_opt);

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
  GetParams();
  std::thread{[this] {this->executor_->spin();}}.detach();
}
bool ExecutorVisionTracking::GetParams()
{
  auto local_share_dir = ament_index_cpp::get_package_share_directory("algorithm_manager");
  auto path = local_share_dir + std::string("/config/VisionTracking.toml");

  if (!cyberdog::common::CyberdogToml::ParseFile(path.c_str(), params_toml_)) {
    ERROR("Params config file is not in toml format");
    return false;
  }

  tracking_keep_distance_ = toml::find<float>(params_toml_, "tracking_keep_distance");
  tracking_relative_pos_ = toml::find<int>(params_toml_, "tracking_relative_pos");
  INFO("tracking_keep_distance_: %2f, tracking_relative_pos_: %d",tracking_keep_distance_,tracking_relative_pos_);
  return true;
}

void ExecutorVisionTracking::Start(const AlgorithmMGR::Goal::ConstSharedPtr goal)
{
  INFO("Start wait(lk)");
  std::unique_lock<std::mutex> lk(start_stop_mutex_);
  INFO("Vision Tracking starting");
  ReportPreparationStatus();
  start_vision_tracking_ = false;
  uint8_t goal_result = StartVisionTracking(
    goal->relative_pos, goal->keep_distance,
    goal->object_tracking);
  if (goal_result != Navigation::Result::NAVIGATION_RESULT_TYPE_ACCEPT) {
    ERROR("ExecutorVisionTracking::Start Error");
    // ReportPreparationFinished(506);
    // task_abort_callback_();
  }
}

void ExecutorVisionTracking::Stop(
  const StopTaskSrv::Request::SharedPtr request,
  StopTaskSrv::Response::SharedPtr response)
{
  INFO("Stop wait(lk)");
  std::unique_lock<std::mutex> lk(start_stop_mutex_);
  (void)request;
  INFO("VisionTracking will stop");
  OnCancel(response);
  INFO("VisionTracking Stoped");
}

void ExecutorVisionTracking::Cancel()
{
  INFO("VisionTracking will cancel");
  OnCancel();
  INFO("VisionTracking canceled");
}

void ExecutorVisionTracking::OnCancel(StopTaskSrv::Response::SharedPtr response)
{
  std::unique_lock<std::mutex> lk(target_tracking_server_mutex_);
  if (target_tracking_goal_handle_ != nullptr) {
    INFO("Will cancel target_tracking_goal_handle_");
    auto future_cancel =
      target_tracking_action_client_->async_cancel_goal(target_tracking_goal_handle_);
    if (target_tracking_server_cv_.wait_for(lk, 10s) == std::cv_status::timeout) {
      WARN("Cancle target_tracking_goal_handle_ timeout");
      cancel_tracking_result_ = false;
    } else {
      INFO("Cancle target_tracking_goal_handle_ success");
      cancel_tracking_result_ = true;
    }
  } else {
    WARN("target_tracking_goal_handle_ is nullptr");
    if (!DeactivateDepsLifecycleNodes(3000)) {
      ERROR("DeactivateDepsLifecycleNodes failed");
    }
  }
  StopReportPreparationThread();
  target_tracking_goal_handle_.reset();
  task_cancle_callback_();
  start_vision_tracking_ = false;
  vision_manager_tracking_ = false;
  if (response == nullptr) {
    return;
  }
  response->result = cancel_tracking_result_ ?
    StopTaskSrv::Response::SUCCESS : StopTaskSrv::Response::FAILED;
}

bool ExecutorVisionTracking::OnlyCancelNavStack()
{
  bool cancel_status;
  std::unique_lock<std::mutex> lk(target_tracking_server_mutex_);
  if (target_tracking_goal_handle_ != nullptr) {
    INFO("Will cancel target_tracking_goal_handle_");
    auto future_cancel =
      target_tracking_action_client_->async_cancel_goal(target_tracking_goal_handle_);
    if (target_tracking_server_cv_.wait_for(lk, 10s) == std::cv_status::timeout) {
      WARN("Cancle target_tracking_goal_handle_ timeout");
      cancel_status = false;
    } else {
      INFO("Cancle target_tracking_goal_handle_ success");
      cancel_status = true;
    }
  } else {
    WARN("target_tracking_goal_handle_ is nullptr");
    cancel_status = true;
  }
  target_tracking_goal_handle_.reset();
  return cancel_status;
}
// TODO(PDF):
bool ExecutorVisionTracking::CallVisionTrackAlgo(bool object_tracking)
{
  auto request = std::make_shared<protocol::srv::AlgoManager::Request>();
  protocol::msg::AlgoList algo;
  // algo.algo_module = protocol::msg::AlgoList::ALGO_FACE;
  // request->algo_enable.push_back(algo);
  if (!object_tracking) {
    algo.algo_module = protocol::msg::AlgoList::ALGO_BODY;
    request->algo_enable.push_back(algo);
    algo.algo_module = protocol::msg::AlgoList::ALGO_REID;
    request->algo_enable.push_back(algo);
  } else {
    algo.algo_module = protocol::msg::AlgoList::ALGO_FOCUS;
    request->algo_enable.push_back(algo);
  }
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
uint8_t ExecutorVisionTracking::StartVisionTracking(
  uint8_t relative_pos, float keep_distance,
  bool object_tracking)
{
  (void)relative_pos;
  (void)keep_distance;
  SetFeedbackCode(500);
  INFO("FeedbackCode: %d", feedback_->feedback_code);
  if (!ActivateDepsLifecycleNodes(this->get_name(), 50000)) {
    ERROR("ActivateDepsLifecycleNodes failed");
    ReportPreparationFinished(506);
    if (!DeactivateDepsLifecycleNodes(3000)) {
      ERROR("DeactivateDepsLifecycleNodes failed");
    }
    task_abort_callback_();
    return Navigation::Result::NAVIGATION_RESULT_TYPE_FAILED;
  }
  if (!CallVisionTrackAlgo(object_tracking)) {
    ERROR("CallVisionTrackAlgo failed");
    ReportPreparationFinished(506);
    if (!DeactivateDepsLifecycleNodes(3000)) {
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
  if (!start_vision_tracking_) {
    ERROR("Should activate the depends node at first");
    res->success = false;
    return;
  }
  // send tracking_object to cyberdog_vision
  if (TrackingClientCallService(client_tracking_object_, req->roi)) {
    INFO("TrackingClientCallService success");
    res->success = true;
  } else {
    ERROR("TrackingClientCallService failed");
    ReportPreparationFinished(507);
    if (!DeactivateDepsLifecycleNodes(3000)) {
      ERROR("DeactivateDepsLifecycleNodes failed");
    }
    task_abort_callback_();
    res->success = false;
    return;
  }
  auto is_action_server_ready =
    target_tracking_action_client_->wait_for_action_server(
    std::chrono::seconds(5));
  if (!is_action_server_ready) {
    ERROR("Tracking target action server is not available.");
    ReportPreparationFinished(507);
    if (!DeactivateDepsLifecycleNodes(3000)) {
      ERROR("DeactivateDepsLifecycleNodes failed");
    }
    res->success = false;
    task_abort_callback_();
    return;
  }
  INFO("is_action_server_ready success");
  // Send the goal pose
  target_tracking_goal_.relative_pos = tracking_relative_pos_;
  target_tracking_goal_.keep_distance = tracking_keep_distance_;
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
    ReportPreparationFinished(507);
    if (!DeactivateDepsLifecycleNodes(3000)) {
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
    ReportPreparationFinished(507);
    if (!DeactivateDepsLifecycleNodes(3000)) {
      ERROR("DeactivateDepsLifecycleNodes failed");
    }
    task_abort_callback_();
    return;
  }
  SetFeedbackCode(502);
  INFO("FeedbackCode: %d", feedback_->feedback_code);
}
void ExecutorVisionTracking::HandleFeedbackCallback(
  TargetTrackingGoalHandle::SharedPtr,
  const std::shared_ptr<const McrTargetTracking::Feedback> feedback)
{
  // INFO("HandleFeedbackCallback: %d", feedback->exception_code);
  if (feedback_->feedback_code == 505) {
    return;
  }
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
      break;
    case rclcpp_action::ResultCode::ABORTED:
      ERROR("Vision Tracking reported aborted");
      target_tracking_goal_handle_.reset();
      feedback_->feedback_code =
        AlgorithmMGR::Feedback::NAVIGATION_FEEDBACK_BASE_TRACKING_EMPTY_TARGET;
      break;
    case rclcpp_action::ResultCode::CANCELED:
      WARN("Vision Tracking reported canceled");
      if (feedback_->feedback_code != 505) {
        if (!DeactivateDepsLifecycleNodes(3000)) {
          ERROR("DeactivateDepsLifecycleNodes failed");
        }
      }
      target_tracking_server_cv_.notify_one();
      break;
    default:
      ERROR("Vision Tracking reported unknown result code");
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
  auto result = client->async_send_request(req);
  if (result.wait_for(std::chrono::milliseconds(5000)) == std::future_status::timeout) {
    ERROR("Cannot Get result TrackingClientCallService");
    return false;
  } else {
    INFO("result.get()->success: %d", result.get()->success);
    return true;
  }
}
void ExecutorVisionTracking::VisionManagerStatusCallback(const TrackingStatusT::SharedPtr msg)
{
  // RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
  switch (msg->status) {
    case TrackingStatusT::STATUS_SELECTING:
      if (vision_manager_tracking_) {
        vision_manager_tracking_ = false;
        SetFeedbackCode(505);
        INFO("FeedbackCode: %d", feedback_->feedback_code);
        if (!OnlyCancelNavStack()) {
          ERROR("OnlyCancelNavStack failed!");
          ReportPreparationFinished(508);
          INFO("FeedbackCode: %d", feedback_->feedback_code);
          if (!DeactivateDepsLifecycleNodes(3000)) {
            ERROR("DeactivateDepsLifecycleNodes failed");
          }
          StopReportPreparationThread();
          task_abort_callback_();
        } else {
          SetFeedbackCode(501);
          INFO("FeedbackCode: %d", feedback_->feedback_code);
        }
      }
      vision_manager_tracking_ = false;
      break;
    case TrackingStatusT::STATUS_TRACKING:
      INFO("TrackingStatusT::STATUS_TRACKING");
      vision_manager_tracking_ = true;
      break;
    default:
      break;
  }
}
}  // namespace algorithm
}  // namespace cyberdog
