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

#include "nav2_control_demo/nav2_localization.hpp"

namespace cyberdog
{
namespace nav2_control_demo
{

LocalizationNode::LocalizationNode() : Node("localization_node")
{
    // Declare this node's parameters
    declare_parameter("slam_type", defualt_slam_type_);
    get_parameter("slam_type", defualt_slam_type_);
}

LocalizationNode::~LocalizationNode()
{
}

void LocalizationNode::HandleStartMappingCallback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    const std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
    if (localization_finished_start_) {
        response->success = true;
        return;
    }

    bool success = false;
    if (defualt_slam_type_ == "lidar") {
        success = StartLidar();
    } else if (defualt_slam_type_ == "vision") {
        success = StartVision();
    }

    if (!success) {
        response->success = false;
    }

    response->success = true;
}

void LocalizationNode::HandleStopMappingCallback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    const std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
    if (localization_finished_stop_) {
        response->success = true;
        return;
    }

    bool success = false;
    if (defualt_slam_type_ == "lidar") {
        success = StoptLidar();
    } else if (defualt_slam_type_ == "vision") {
        success = StopVision();
    }

    if (!success) {
        response->success = false;
    }

    response->success = true;
}


bool LocalizationNode::StartLidar()
{
    // Control lidar relocalization turn on
  if (start_client_ == nullptr) {
    start_client_ = std::make_shared<nav2_util::ServiceClient<std_srvs::srv::SetBool>>(
      "start_location", shared_from_this());
  }

  // Wait service
  bool connect = start_client_->wait_for_service(std::chrono::seconds(2));
  if (!connect) {
    RCLCPP_ERROR(this->get_logger(), "Waiting for the service(start_location). but cannot connect the service.");
    return false;
  }

  // Set request data
  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = true;

  // Send request
  // return start_->invoke(request, response);
  bool result = false;
  try {
    RCLCPP_INFO(this->get_logger(), "EnableRelocalization(): Trying to get service_mutex_");
    std::lock_guard<std::mutex> lock(service_mutex_);
    RCLCPP_INFO(this->get_logger(), "EnableRelocalization(): Success to get service_mutex_");
    auto future_result = start_client_->invoke(request, std::chrono::seconds(50));
    result = future_result->success;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "%s", e.what());
  }
  return result;
}

bool LocalizationNode::StartVision()
{
    return true;
}

bool LocalizationNode::StoptLidar()
{
    // Control lidar relocalization turn off
    if (stop_client_ == nullptr) {
        stop_client_ = std::make_shared<nav2_util::ServiceClient<std_srvs::srv::SetBool>>(
        "stop_location", shared_from_this());
    }

    // Wait service
    bool connect = stop_client_->wait_for_service(std::chrono::seconds(2));
    if (!connect) {
        RCLCPP_ERROR(this->get_logger(),"Waiting for the service(stop_location). but cannot connect the service.");
        return false;
    }

    // Set request data
    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = true;

    // Send request
    // return start_->invoke(request, response);
    bool result = false;
    try {
        RCLCPP_INFO(this->get_logger(),"DisableRelocalization(): Trying to get service_mutex_");
        std::lock_guard<std::mutex> lock(service_mutex_);
        RCLCPP_INFO_SKIPFIRST(this->get_logger(),"DisableRelocalization(): Success to get service_mutex_");
        auto future_result = stop_client_->invoke(request, std::chrono::seconds(10));
        result = future_result->success;
    } catch (const std::exception & e) {
       RCLCPP_ERROR(this->get_logger(), "%s", e.what());
    }
    return result;
}

bool LocalizationNode::StopVision()
{
    return true;
}

}  // namespace nav2_control_demo
}  // namespace cyberdog
