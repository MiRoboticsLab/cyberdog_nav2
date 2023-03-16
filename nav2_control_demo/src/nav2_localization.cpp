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

#include <memory>
#include "rclcpp/rclcpp.hpp"

namespace cyberdog
{
namespace nav2_control_demo
{

LocalizationNode::LocalizationNode() : Node("nav2_localization_node")
{
    // Declare this node's parameters
    // declare_parameter("slam_type", defualt_slam_type_);
    // get_parameter("slam_type", defualt_slam_type_);

    lidar_start_sub_ = this->create_subscription<std_msgs::msg::Bool>("lidar_start_localization", 1, 
        std::bind(&LocalizationNode::HandleStartLidarLocalizationCallback, this, std::placeholders::_1));

    lidar_stop_sub_ = this->create_subscription<std_msgs::msg::Bool>("lidar_stop_localization", 1, 
        std::bind(&LocalizationNode::HandleStopLidarLocalizationCallback, this, std::placeholders::_1));

    vision_start_sub_ = this->create_subscription<std_msgs::msg::Bool>("vision_start_localization", 1, 
        std::bind(&LocalizationNode::HandleStartVisionLocalizationCallback, this, std::placeholders::_1));

    vision_stop_sub_ = this->create_subscription<std_msgs::msg::Bool>("vision_stop_localization", 1, 
        std::bind(&LocalizationNode::HandleStopVisionLocalizationCallback, this, std::placeholders::_1));
}

LocalizationNode::~LocalizationNode()
{
}

void LocalizationNode::HandleStartLidarLocalizationCallback(
    const std::shared_ptr<std_msgs::msg::Bool> msg)
{
    if (!msg->data) {
        return;
    }

    bool ret = StartLidar();
    if (!ret) {
        RCLCPP_ERROR(this->get_logger(), "start lidar localization failed");
    } else {
        RCLCPP_INFO(this->get_logger(), "start lidar localization success");
    }
}

void LocalizationNode::HandleStopLidarLocalizationCallback(
    const std::shared_ptr<std_msgs::msg::Bool> msg)
{
    if (!msg->data) {
        return;
    }

    bool ret = StopLidar();
    if (!ret) {
        RCLCPP_ERROR(this->get_logger(), "stop lidar localization failed");
    } else {
        RCLCPP_INFO(this->get_logger(), "stop lidar localization success");
    }
}

void LocalizationNode::HandleStartVisionLocalizationCallback(
    const std::shared_ptr<std_msgs::msg::Bool> msg)
{
    if (!msg->data) {
        return;
    }

    bool ret = StartVision();
    if (!ret) {
        RCLCPP_ERROR(this->get_logger(), "start vision localization failed");
    } else {
        RCLCPP_INFO(this->get_logger(), "start vision localization success");
    }
}

void LocalizationNode::HandleStopVisionLocalizationCallback(
    const std::shared_ptr<std_msgs::msg::Bool> msg)
{
    if (!msg->data) {
        return;
    }

    bool ret = StopVision();
    if (!ret) {
        RCLCPP_ERROR(this->get_logger(), "stop vision localization failed");
    } else {
        RCLCPP_INFO(this->get_logger(), "stop vision localization success");
    }
}

bool LocalizationNode::StartLidar()
{
    // Control lidar relocalization turn on
    if (lidar_start_client_ == nullptr) {
        lidar_start_client_ = std::make_shared<nav2_util::ServiceClient<std_srvs::srv::SetBool>>(
        "start_location", shared_from_this());
    }

    // Wait service
    bool connect = lidar_start_client_->wait_for_service(std::chrono::seconds(2));
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
        auto future_result = lidar_start_client_->invoke(request, std::chrono::seconds(50));
        result = future_result->success;
    } catch (const std::exception & e) {
        RCLCPP_ERROR(this->get_logger(), "%s", e.what());
    }
    return result;
}

bool LocalizationNode::StartVision()
{
    if (vision_start_client_ == nullptr) {
        vision_start_client_ = std::make_shared<nav2_util::ServiceClient<std_srvs::srv::SetBool>>(
        "start_vins_location", shared_from_this());
    }

    // Wait service
    bool connect = vision_start_client_->wait_for_service(std::chrono::seconds(2));
    if (!connect) {
        RCLCPP_ERROR(this->get_logger(), "Waiting for the service(start_vins_location) timeout");
        return false;
    }

    // Set request data
    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = true;

    // Send request
    // return start_->invoke(request, response);
    bool result = false;

    try {
        RCLCPP_INFO(this->get_logger(), "EnableRelocalization(): Trying to get service mutex");
        std::lock_guard<std::mutex> lock(service_mutex_);
        RCLCPP_INFO(this->get_logger(), "EnableRelocalization(): Success to get service mutex");
        auto future_result = vision_start_client_->invoke(request, std::chrono::seconds(50));
        result = future_result->success;
    } catch (const std::exception & e) {
        RCLCPP_ERROR(this->get_logger(), "%s", e.what());
    }
    return result;
}

bool LocalizationNode::StopLidar()
{
    // Control lidar relocalization turn off
    if (lidar_stop_client_ == nullptr) {
        lidar_stop_client_ = std::make_shared<nav2_util::ServiceClient<std_srvs::srv::SetBool>>(
        "stop_location", shared_from_this());
    }

    // Wait service
    bool connect = lidar_stop_client_->wait_for_service(std::chrono::seconds(2));
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
        RCLCPP_INFO(this->get_logger(), "DisableRelocalization(): Trying to get service_mutex_");
        std::lock_guard<std::mutex> lock(service_mutex_);
        RCLCPP_INFO_SKIPFIRST(this->get_logger(),"DisableRelocalization(): Success to get service_mutex_");
        auto future_result = lidar_stop_client_->invoke(request, std::chrono::seconds(10));
        result = future_result->success;
    } catch (const std::exception & e) {
       RCLCPP_ERROR(this->get_logger(), "%s", e.what());
    }
    return result;
}

bool LocalizationNode::StopVision()
{
    if (vision_stop_client_ == nullptr) {
        vision_stop_client_ = std::make_shared<nav2_util::ServiceClient<std_srvs::srv::SetBool>>(
        "stop_vins_location", shared_from_this());
    }

    // Wait service
    bool connect = vision_stop_client_->wait_for_service(std::chrono::seconds(2));
    if (!connect) {
        RCLCPP_ERROR(this->get_logger(), "Waiting for the service(stop_vins_location) timeout");
        return false;
    }

    // Set request data
    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = true;

    // Send request
    // return start_->invoke(request, response);
    bool result = false;

    try {
        RCLCPP_INFO(this->get_logger(), "DisableRelocalization(): Trying to get service mutex");
        std::lock_guard<std::mutex> lock(service_mutex_);
        RCLCPP_INFO(this->get_logger(), "DisableRelocalization(): Success to get service mutex");
        auto future_result = vision_stop_client_->invoke(request, std::chrono::seconds(10));
        result = future_result->success;
    } catch (const std::exception & e) {
        RCLCPP_ERROR(this->get_logger(), "%s", e.what());
    }
    return result;
}

}  // namespace nav2_control_demo
}  // namespace cyberdog

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<cyberdog::nav2_control_demo::LocalizationNode>();
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();

    return 0;
}
