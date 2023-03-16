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

#ifndef CYBERDOG_NAV2_CONTROL_DEMO_NAV2_LOCALIZATION_HPP_
#define CYBERDOG_NAV2_CONTROL_DEMO_NAV2_LOCALIZATION_HPP_

#include <memory>
#include <vector>
#include <unordered_map>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "visualization/srv/stop.hpp"
#include "nav2_util/lifecycle_service_client.hpp"
#include "std_msgs/msg/bool.hpp"

namespace cyberdog
{
namespace nav2_control_demo
{

class LocalizationNode : public rclcpp::Node
{
public:
    LocalizationNode();
    ~LocalizationNode();

private:
    void HandleStartLidarLocalizationCallback(const std::shared_ptr<std_msgs::msg::Bool> msg);
    void HandleStopLidarLocalizationCallback(const std::shared_ptr<std_msgs::msg::Bool> msg);
    void HandleStartVisionLocalizationCallback(const std::shared_ptr<std_msgs::msg::Bool> msg);
    void HandleStopVisionLocalizationCallback(const std::shared_ptr<std_msgs::msg::Bool> msg);

    bool StartLidar();
    bool StartVision();
    bool StopLidar();
    bool StopVision();

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr lidar_start_sub_{nullptr};
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr lidar_stop_sub_{nullptr};
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr vision_start_sub_{nullptr};
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr vision_stop_sub_{nullptr};

    std::shared_ptr<nav2_util::ServiceClient<std_srvs::srv::SetBool>> lidar_start_client_ {nullptr};
    std::shared_ptr<nav2_util::ServiceClient<std_srvs::srv::SetBool>> lidar_stop_client_ {nullptr};
    std::shared_ptr<nav2_util::ServiceClient<std_srvs::srv::SetBool>> vision_start_client_ {nullptr};
    std::shared_ptr<nav2_util::ServiceClient<std_srvs::srv::SetBool>> vision_stop_client_ {nullptr};

    std::string defualt_slam_type_;

    std::mutex service_mutex_;
    bool localization_finished_start_{false};
    bool localization_finished_stop_{false};
};

}  // namespace nav2_control_demo
}  // namespace cyberdog

#endif  // CYBERDOG_NAV2_CONTROL_DEMO_NAV2_LOCALIZATION_HPP_