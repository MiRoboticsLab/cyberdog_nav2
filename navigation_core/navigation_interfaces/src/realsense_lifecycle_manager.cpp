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

#include <string>
#include <memory>

#include "motion_core/realsense_lifecycle_manager.hpp"
#include "cyberdog_common/cyberdog_log.hpp"

namespace carpo_navigation
{

// which node to handle
static constexpr char const * lifecycle_node = "camera/camera";

// In this demo, we use get_state and change_state
// and thus the two service topics are:
// camera/camera/get_state
// camera/camera/change_state
static constexpr char const * node_get_state_topic = "camera/camera/get_state";
static constexpr char const * node_change_state_topic = "camera/camera/change_state";

template<typename FutureT, typename WaitTimeT>
std::future_status wait_for_result(FutureT & future, WaitTimeT time_to_wait)
{
  auto end = std::chrono::steady_clock::now() + time_to_wait;
  std::chrono::milliseconds wait_period(100);
  std::future_status status = std::future_status::timeout;
  do {
    auto now = std::chrono::steady_clock::now();
    auto time_left = end - now;
    if (time_left <= std::chrono::seconds(0)) {break;}
    status = future.wait_for((time_left < wait_period) ? time_left : wait_period);
  } while (rclcpp::ok() && status != std::future_status::ready);
  return status;
}

RealSenseLifecycleServiceClient::RealSenseLifecycleServiceClient(const std::string & node_name)
: Node(node_name)
{
  callback_group_ = this->create_callback_group(
    rclcpp::CallbackGroupType::Reentrant, false);

  // callback_group_executor_.add_callback_group(callback_group_, this->get_node_base_interface());

  client_get_state_ = this->create_client<lifecycle_msgs::srv::GetState>(
    node_get_state_topic,
    rmw_qos_profile_services_default,
    callback_group_);

  client_change_state_ = this->create_client<lifecycle_msgs::srv::ChangeState>(
    node_change_state_topic,
    rmw_qos_profile_services_default,
    callback_group_);

  init();
}

RealSenseLifecycleServiceClient::~RealSenseLifecycleServiceClient()
{
  Cleanup();
}

void RealSenseLifecycleServiceClient::init()
{
  client_get_state_ = this->create_client<lifecycle_msgs::srv::GetState>(
    node_get_state_topic);

  client_change_state_ = this->create_client<lifecycle_msgs::srv::ChangeState>(
    node_change_state_topic);
}

unsigned int RealSenseLifecycleServiceClient::GetState(std::chrono::seconds time_out)
{
  auto request = std::make_shared<lifecycle_msgs::srv::GetState::Request>();

  if (!client_get_state_->wait_for_service(time_out)) {
    ERROR("Service %s is not available.", client_get_state_->get_service_name());
    return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
  }

  auto future_result = client_get_state_->async_send_request(request);
  auto future_status = wait_for_result(future_result, time_out);
  INFO("GetState() future_status : %d", future_status);

  if (future_status == std::future_status::ready) {
    INFO("Node %s has current state %s.",
      lifecycle_node, future_result.get()->current_state.label.c_str());
    return future_result.get()->current_state.id;
  }

 RCLCPP_ERROR(
      get_logger(), "Failed to get current state for node %s", lifecycle_node);
  return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
}

bool RealSenseLifecycleServiceClient::ChangeState(
  std::uint8_t transition, std::chrono::seconds time_out)
{
  auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
  request->transition.id = transition;

  if (!client_change_state_->wait_for_service(time_out)) {
    ERROR("Service %s is not available.", client_change_state_->get_service_name());
    return false;
  }

  // We send the request with the transition we want to invoke.
  auto future_result = client_change_state_->async_send_request(request);
  auto future_status = wait_for_result(future_result, time_out);
  INFO("ChangeState() future_status : %d", future_status);

  if (future_status == std::future_status::ready) {
    INFO("Transition %d successfully triggered.", static_cast<int>(transition));
    return true;
  }

  WARN("Failed to trigger transition %u", static_cast<unsigned int>(transition));
  return false;
}

bool RealSenseLifecycleServiceClient::Startup()
{
  if (!ChangeState(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE)) {
    INFO("RealSense set configure state error.");
  }

  // activate
  if (!ChangeState(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE)) {
    INFO("RealSense set activate state error.");
    return false;
  }
  return true;
}

bool RealSenseLifecycleServiceClient::Pause()
{
  // deactivate
  if (!ChangeState(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE)) {
    INFO("RealSense set deactivate state error.");
    return false;
  }
  return true;
}

bool RealSenseLifecycleServiceClient::Cleanup()
{
  // cleanup
  if (!ChangeState(lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP)) {
    INFO("RealSense set cleanup state error.");
    return false;
  }
  return true;
}

}  // namespace carpo_navigation
