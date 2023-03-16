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

    lidar_start_sub_ = this->create_subscription<std_msgs::msg::Bool>("lidar_start_mapping", 1, 
        std::bind(&MappingNode::HandleLidarStartMappingCallback, this, std::placeholders::_1));

    lidar_stop_sub_ = this->create_subscription<std_msgs::msg::Bool>("lidar_stop_mapping", 1, 
        std::bind(&MappingNode::HandleLidarStopMappingCallback, this, std::placeholders::_1));

    vision_start_sub_ = this->create_subscription<std_msgs::msg::Bool>("vision_start_mapping", 1, 
        std::bind(&MappingNode::HandleVisionStartMappingCallback, this, std::placeholders::_1));

    vision_stop_sub_ = this->create_subscription<std_msgs::msg::Bool>("vision_stop_mapping", 1, 
        std::bind(&MappingNode::HandleVisionStopMappingCallback, this, std::placeholders::_1));
}

MappingNode::~MappingNode()
{
}

void MappingNode::HandleLidarStartMappingCallback(const std::shared_ptr<std_msgs::msg::Bool> msg)
{
    if (!msg->data) {
        return;
    }

    bool success = StartLidarMapping();
    if (!success) {
        RCLCPP_ERROR(this->get_logger(), "start lidar mapping error.");
    } else {
        RCLCPP_INFO(this->get_logger(), "start lidar mapping success.");
    }
}

void MappingNode::HandleLidarStopMappingCallback(const std::shared_ptr<std_msgs::msg::Bool> msg)
{
    if (!msg->data) {
        return;
    }

    std::string mapname = "map";
    bool success = StopLidarMapping(mapname);
    if (!success) {
        RCLCPP_ERROR(this->get_logger(), "save lidar map error");
    } else {
        RCLCPP_INFO(this->get_logger(), "save lidar mapping success.");
    }
}

void MappingNode::HandleVisionStartMappingCallback(const std::shared_ptr<std_msgs::msg::Bool> msg)
{
    if (!msg->data) {
        return;
    }

    bool success = StartVisionMapping();
    if (!success) {
        RCLCPP_ERROR(this->get_logger(), "start vision mapping error.");
    } else {
        RCLCPP_INFO(this->get_logger(), "start vision mapping success.");
    }

}

void MappingNode::HandleVisionStopMappingCallback(const std::shared_ptr<std_msgs::msg::Bool> msg)
{
    if (!msg->data) {
        return;
    }

    std::string mapname = "map";
    bool success = StopVisionMapping(mapname);
    if (!success) {
        RCLCPP_ERROR(this->get_logger(), "save vision map error");
    } else {
        RCLCPP_INFO(this->get_logger(), "stop vision mapping success.");
    }
}

bool MappingNode::StartLidarMapping()
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

bool MappingNode::StopLidarMapping(const std::string& map_filename)
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

bool MappingNode::StartVisionMapping()
{
    if (vision_start_client_ == nullptr) {
        vision_start_client_ = std::make_shared<nav2_util::ServiceClient<std_srvs::srv::SetBool>>(
        "start_vins_mapping", shared_from_this());
    }

    // Wait service
    bool connect = vision_start_client_->wait_for_service(std::chrono::seconds(2));
    if (!connect) {
        RCLCPP_ERROR(this->get_logger(), "Waiting for the service(start_vins_mapping) timeout.");
        return false;
    }

    // Set request data
    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = true;

    // Send request
    // return start_->invoke(request, response);
    bool result = false;
    try {
        std::lock_guard<std::mutex> lock(service_mutex_);
        auto future_result = vision_start_client_->invoke(request, std::chrono::seconds(5));
        result = future_result->success;
    } catch (const std::exception & e) {
        RCLCPP_ERROR(this->get_logger(), "%s", e.what());
    }

  return result;
    return true;
}

bool MappingNode::StopVisionMapping(const std::string& map_filename)
{
    if (vision_stop_client_ == nullptr) {
        vision_stop_client_ = std::make_shared<nav2_util::ServiceClient<MapRequest>>(
        "stop_vins_mapping", shared_from_this());
    }

    // Wait service
    bool connect = vision_stop_client_->wait_for_service(std::chrono::seconds(2));
    if (!connect) {
        RCLCPP_ERROR(this->get_logger(), "Waiting for the service(stop_vins_mapping) timeout.");
        return false;
    }

    // Set request data
    auto request = std::make_shared<MapRequest::Request>();
    if (map_filename.empty()) {
        RCLCPP_WARN(this->get_logger(), "User set map name is empty");
        request->finish = false;
        request->map_name = "";
    } else {
        RCLCPP_INFO(this->get_logger(), "Saved map building filename: %s", map_filename.c_str());
        request->finish = true;
        request->map_name = map_filename;
    }

    // Send request
    bool result = false;
    try {
        std::lock_guard<std::mutex> lock(service_mutex_);
        auto future_result = vision_stop_client_->invoke(request, std::chrono::seconds(10));
        result = future_result->success;
    } catch (const std::exception & e) {
        RCLCPP_ERROR(this->get_logger(), "%s", e.what());
    }

    return result;
}

}  // namespace nav2_control_demo
}  // namespace cyberdog
