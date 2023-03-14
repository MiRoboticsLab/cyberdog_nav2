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

#include "nav2_control_demo/nav2_mapping.hpp"

namespace cyberdog
{
namespace nav2_control_demo
{

MappingNode::MappingNode() : Node("mapping_node")
{
    // Declare this node's parameters
    declare_parameter("map_filename", defualt_map_filename_);
    get_parameter("map_filename", defualt_map_filename_);

    stop_server_ = this->create_service<std_srvs::srv::SetBool>(
        "stop_mapping", std::bind(
        &MappingNode::HandleStartMappingCallback, this,
        std::placeholders::_1, std::placeholders::_2));

    start_server_ = this->create_service<std_srvs::srv::SetBool>(
        "start_mapping", std::bind(
        &MappingNode::HandleStopMappingCallback, this,
        std::placeholders::_1, std::placeholders::_2));
}

MappingNode::~MappingNode()
{
}

void MappingNode::HandleStartMappingCallback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    const std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
    if (mapping_start_) {
        RCLCPP_WARN(this->get_logger(), "activate dependency lifecycle nodes haved success.");
        response->success = true;
        return;
    }

    response->success = true;
}

void MappingNode::HandleStopMappingCallback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    const std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
    if (!mapping_start_ || mapping_stop_) {
        RCLCPP_WARN(this->get_logger(), "deactivate dependency lifecycle nodes haved success.");
        response->success = true;
        return;
    }

    std::string mapname = "map";
    bool success = StopMapping(mapname);
    if (!success) {
        RCLCPP_ERROR(this->get_logger(), "save map error");
        response->success = false;
        return;
    }

    response->success = true;
}


bool MappingNode::StartMapping()
{
  if (mapping_start_client_ == nullptr) {
    mapping_start_client_ = std::make_shared<nav2_util::ServiceClient<std_srvs::srv::SetBool>>(
      "start_mapping", shared_from_this());
  }

  // Wait service
  bool connect = mapping_start_client_->wait_for_service(std::chrono::seconds(2));
  if (!connect) {
    RCLCPP_ERROR(this->get_logger(), "Waiting for the service(start_mapping), but cannot connect the service.");
    return false;
  }

  // Set request data
  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  auto response = std::make_shared<std_srvs::srv::SetBool::Response>();
  request->data = true;

  // Send request
  // return start_->invoke(request, response);
  bool result = false;
  try {
    std::lock_guard<std::mutex> lock(service_mutex_);
    auto future_result = mapping_start_client_->invoke(request, std::chrono::seconds(5));
    result = future_result->success;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "%s", e.what());
  }
  return result;
}

bool MappingNode::StopMapping(const std::string& map_filename)
{
    if (mapping_stop_client_ == nullptr) {
        mapping_stop_client_ = std::make_shared<nav2_util::ServiceClient<visualization::srv::Stop>>(
        "stop_mapping", shared_from_this());
    }

    // Wait service
    bool connect = mapping_stop_client_->wait_for_service(std::chrono::seconds(2));
    if (!connect) {
        RCLCPP_ERROR(this->get_logger(),"Waiting for the service(stop_mapping), but cannot connect the service.");
        return false;
    }

    // Set request data
    auto request = std::make_shared<visualization::srv::Stop::Request>();
    if (map_filename.empty()) {
        request->finish = false;
        RCLCPP_WARN(this->get_logger(),"User set map name is empty");
    } else {
        request->finish = true;
        request->map_name = map_filename;
        RCLCPP_INFO(this->get_logger(),"Saved map building filename: %s", map_filename.c_str());
    }

    // Send request
    // return stop_->invoke(request, response);
    bool result = false;
    try {
        std::lock_guard<std::mutex> lock(service_mutex_);
        auto future_result = mapping_stop_client_->invoke(request, std::chrono::seconds(15));
        result = future_result->success;
    } catch (const std::exception & e) {
        RCLCPP_ERROR(this->get_logger(), "%s", e.what());
    }

    return result;
}

}  // namespace nav2_control_demo
}  // namespace cyberdog
