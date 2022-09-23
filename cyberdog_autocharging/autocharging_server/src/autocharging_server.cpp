// Copyright 2019 Open Source Robotics Foundation, Inc.
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

#include <functional>
#include <memory>
#include <thread>

#include "autocharging_server/autocharging_server.hpp"

namespace cyberdog
{
AutoChargingServer::AutoChargingServer(const rclcpp::NodeOptions & options)
: Node("auto_charging_server", options), stage2_goal_done_(false), stage3_goal_done_(false)
{
  using namespace std::placeholders;

  this->action_server_ = rclcpp_action::create_server<AutoChargingT>(
    this->get_node_base_interface(),
    this->get_node_clock_interface(),
    this->get_node_logging_interface(),
    this->get_node_waitables_interface(),
    "autocharging",
    std::bind(&AutoChargingServer::handle_goal, this, _1, _2),
    std::bind(&AutoChargingServer::handle_cancel, this, _1),
    std::bind(&AutoChargingServer::handle_accepted, this, _1));

  this->client_navtopose_ptr_ = rclcpp_action::create_client<NavigateToPoseT>(
    this->get_node_base_interface(),
    this->get_node_graph_interface(),
    this->get_node_logging_interface(),
    this->get_node_waitables_interface(),
    "navigate_to_pose");

  this->client_stage2_ptr_ = rclcpp_action::create_client<AutomaticRechargeT>(
    this->get_node_base_interface(),
    this->get_node_graph_interface(),
    this->get_node_logging_interface(),
    this->get_node_waitables_interface(),
    "automatic_recharge");

  this->client_seat_adjust_ptr_ = rclcpp_action::create_client<SeatAdjustT>(
    this->get_node_base_interface(),
    this->get_node_graph_interface(),
    this->get_node_logging_interface(),
    this->get_node_waitables_interface(),
    "seatadjust");

  RCLCPP_INFO(this->get_logger(), "autocharging server is ready");
}
// This section is for the top server interface
rclcpp_action::GoalResponse AutoChargingServer::handle_goal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const AutoChargingT::Goal> goal)
{
  RCLCPP_INFO(this->get_logger(), "Received goal request with start %d", goal->start);
  (void)uuid;
  // Let's reject sequences that are over 9000
  if (goal->start == 0) {
    return rclcpp_action::GoalResponse::REJECT;
  }
  this->goal_pose = goal->pose;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse AutoChargingServer::handle_cancel(
  const std::shared_ptr<GoalHandleAutoCharging> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void AutoChargingServer::execute(const std::shared_ptr<GoalHandleAutoCharging> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Executing goal");
  rclcpp::Rate loop_rate(1);
  const auto goal = goal_handle->get_goal();
  auto feedback = std::make_shared<AutoChargingT::Feedback>();
  auto result = std::make_shared<AutoChargingT::Result>();
  int i = 0;
  bool stage2_start = false;
  stage1_send_goal(this->goal_pose);
  // for (int i = 1; (i < 10) && rclcpp::ok(); ++i) {
  while (!this->stage1_goal_done_ || !this->stage2_goal_done_) {
    // Check if there is a cancel request
    if (goal_handle->is_canceling()) {
      result->result = result->AUTOCHARGING_RESULT_TYPE_CANCEL;
      goal_handle->canceled(result);
      RCLCPP_INFO(this->get_logger(), "Goal Canceled");
      return;
    }
    feedback->charging_stage = i;
    goal_handle->publish_feedback(feedback);
    RCLCPP_INFO(this->get_logger(), "execute count: %d", i);
    loop_rate.sleep();
    i++;
  }
  // Check if goal is done
  if (rclcpp::ok()) {
    result->result = result->AUTOCHARGING_RESULT_TYPE_SUCCESS;
    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "Goal Succeeded");
  }
}

void AutoChargingServer::handle_accepted(const std::shared_ptr<GoalHandleAutoCharging> goal_handle)
{
  using namespace std::placeholders;
  // this needs to return quickly to avoid blocking the executor, so spin up a new thread
  std::thread{std::bind(&AutoChargingServer::execute, this, _1), goal_handle}.detach();
}

// This section is for the stage1 client interface
void AutoChargingServer::stage1_goal_response_callback(
  GoalHandleNavigateToPose::SharedPtr goal_handle)
{
  if (!goal_handle) {
    RCLCPP_ERROR(this->get_logger(), "stage1 Goal was rejected by server");
  } else {
    RCLCPP_INFO(this->get_logger(), "stage1 Goal accepted by server, waiting for result");
  }
}

void AutoChargingServer::stage1_feedback_callback(
  GoalHandleNavigateToPose::SharedPtr,
  const std::shared_ptr<const NavigateToPoseT::Feedback> feedback)
{
  RCLCPP_INFO(
    this->get_logger(),
    "stage1 feedback distance_remaining: %f",
    feedback->distance_remaining);
}

void AutoChargingServer::stage1_result_callback(
  const GoalHandleNavigateToPose::WrappedResult & result)
{
  this->stage1_goal_done_ = true;
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(this->get_logger(), "stage1 Result received, next stage will start.");
      this->stage2_send_goal();
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(this->get_logger(), "stage1 Goal was aborted");
      return;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(this->get_logger(), "stage1 Goal was canceled");
      return;
    default:
      RCLCPP_ERROR(this->get_logger(), "stage1 Unknown result code");
      return;
  }
}

void AutoChargingServer::stage1_send_goal(const geometry_msgs::msg::PoseStamped pose)
{
  using namespace std::placeholders;

  this->stage1_goal_done_ = false;

  if (!this->client_navtopose_ptr_) {
    RCLCPP_ERROR(this->get_logger(), "Action client not initialized");
  }

  if (!this->client_navtopose_ptr_->wait_for_action_server(std::chrono::seconds(10))) {
    RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
    this->stage1_goal_done_ = true;
    return;
  }

  auto goal_msg = NavigateToPoseT::Goal();
  goal_msg.pose = pose;

  RCLCPP_INFO(this->get_logger(), "Sending goal");

  auto send_goal_options = rclcpp_action::Client<NavigateToPoseT>::SendGoalOptions();
  send_goal_options.goal_response_callback =
    std::bind(&AutoChargingServer::stage1_goal_response_callback, this, _1);
  send_goal_options.feedback_callback =
    std::bind(&AutoChargingServer::stage1_feedback_callback, this, _1, _2);
  send_goal_options.result_callback =
    std::bind(&AutoChargingServer::stage1_result_callback, this, _1);
  auto goal_handle_future =
    this->client_navtopose_ptr_->async_send_goal(goal_msg, send_goal_options);
}
// This section is for the stage2 client interface
void AutoChargingServer::stage2_goal_response_callback(
  GoalHandleAutomaticRecharge::SharedPtr goal_handle)
{
  if (!goal_handle) {
    RCLCPP_ERROR(this->get_logger(), "stage2 Goal was rejected by server");
  } else {
    RCLCPP_INFO(this->get_logger(), "stage2 Goal accepted by server, waiting for result");
  }
}

void AutoChargingServer::stage2_feedback_callback(
  GoalHandleAutomaticRecharge::SharedPtr,
  const std::shared_ptr<const AutomaticRechargeT::Feedback> feedback)
{
  RCLCPP_INFO(
    this->get_logger(),
    "stage2 feedback current_distance: %f",
    feedback->current_distance);
}

void AutoChargingServer::stage2_result_callback(
  const GoalHandleAutomaticRecharge::WrappedResult & result)
{
  this->stage2_goal_done_ = true;
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(this->get_logger(), "stage2 Result received, next stage will start.");
      this->stage3_send_goal();
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(this->get_logger(), "stage2 Goal was aborted");
      return;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(this->get_logger(), "stage2 Goal was canceled");
      return;
    default:
      RCLCPP_ERROR(this->get_logger(), "stage2 Unknown result code");
      return;
  }
}

void AutoChargingServer::stage2_send_goal()
{
  using namespace std::placeholders;

  this->stage2_goal_done_ = false;

  if (!this->client_stage2_ptr_) {
    RCLCPP_ERROR(this->get_logger(), "Action client not initialized");
  }

  if (!this->client_stage2_ptr_->wait_for_action_server(std::chrono::seconds(10))) {
    RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
    this->stage2_goal_done_ = true;
    return;
  }

  auto goal_msg = AutomaticRechargeT::Goal();
  goal_msg.behavior_tree = "";

  RCLCPP_INFO(this->get_logger(), "Sending goal");

  auto send_goal_options = rclcpp_action::Client<AutomaticRechargeT>::SendGoalOptions();
  send_goal_options.goal_response_callback =
    std::bind(&AutoChargingServer::stage2_goal_response_callback, this, _1);
  send_goal_options.feedback_callback =
    std::bind(&AutoChargingServer::stage2_feedback_callback, this, _1, _2);
  send_goal_options.result_callback =
    std::bind(&AutoChargingServer::stage2_result_callback, this, _1);
  auto goal_handle_future = this->client_stage2_ptr_->async_send_goal(goal_msg, send_goal_options);
}

// This section is for the seat_adjust stage client interface
void AutoChargingServer::stage3_goal_response_callback(GoalHandleSeatAdjust::SharedPtr goal_handle)
{
  if (!goal_handle) {
    RCLCPP_ERROR(this->get_logger(), "stage3 Goal was rejected by server");
  } else {
    RCLCPP_INFO(this->get_logger(), "stage3 Goal accepted by server, waiting for result");
  }
}

void AutoChargingServer::stage3_feedback_callback(
  GoalHandleSeatAdjust::SharedPtr,
  const std::shared_ptr<const SeatAdjustT::Feedback> feedback)
{
  RCLCPP_INFO(
    this->get_logger(),
    "stage3 feedback count: %d",
    feedback->count);
}

void AutoChargingServer::stage3_result_callback(const GoalHandleSeatAdjust::WrappedResult & result)
{
  this->stage3_goal_done_ = true;
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(this->get_logger(), "stage3 Result received, next stage will start.");
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(this->get_logger(), "stage3 Goal was aborted");
      return;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(this->get_logger(), "stage3 Goal was canceled");
      return;
    default:
      RCLCPP_ERROR(this->get_logger(), "stage3 Unknown result code");
      return;
  }
}

void AutoChargingServer::stage3_send_goal()
{
  using namespace std::placeholders;

  this->stage3_goal_done_ = false;

  if (!this->client_seat_adjust_ptr_) {
    RCLCPP_ERROR(this->get_logger(), "Action client not initialized");
  }

  if (!this->client_seat_adjust_ptr_->wait_for_action_server(std::chrono::seconds(10))) {
    RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
    this->stage3_goal_done_ = true;
    return;
  }

  auto goal_msg = SeatAdjustT::Goal();
  goal_msg.start = goal_msg.SEATADJUST_GOAL_TYPE_START;

  RCLCPP_INFO(this->get_logger(), "Sending goal");

  auto send_goal_options = rclcpp_action::Client<SeatAdjustT>::SendGoalOptions();
  send_goal_options.goal_response_callback =
    std::bind(&AutoChargingServer::stage3_goal_response_callback, this, _1);
  send_goal_options.feedback_callback =
    std::bind(&AutoChargingServer::stage3_feedback_callback, this, _1, _2);
  send_goal_options.result_callback =
    std::bind(&AutoChargingServer::stage3_result_callback, this, _1);
  auto goal_handle_future = this->client_seat_adjust_ptr_->async_send_goal(
    goal_msg,
    send_goal_options);
}
}  // namespace cyberdog

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto action_server = std::make_shared<cyberdog::AutoChargingServer>();

  rclcpp::spin(action_server);

  rclcpp::shutdown();
  return 0;
}
