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

#ifndef CYBERDOG_NAV2_CONTROL_DEMO_NAV2_MAPPING_HPP_
#define CYBERDOG_NAV2_CONTROL_DEMO_NAV2_MAPPING_HPP_

#include <memory>
#include <vector>
#include <unordered_map>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_msgs/msg/bool.hpp"

#include "visualization/srv/stop.hpp"
#include "nav2_util/lifecycle_service_client.hpp"

namespace cyberdog
{
namespace nav2_control_demo
{

class MappingNode : public rclcpp::Node
{
public:
    MappingNode();
    ~MappingNode();

private:
    void HandleLidarStartMappingCallback(const std::shared_ptr<std_msgs::msg::Bool> msg);
    void HandleLidarStopMappingCallback(const std::shared_ptr<std_msgs::msg::Bool> msg);
    void HandleVisionStartMappingCallback(const std::shared_ptr<std_msgs::msg::Bool> msg);
    void HandleVisionStopMappingCallback(const std::shared_ptr<std_msgs::msg::Bool> msg);

    bool StartLidarMapping();
    bool StopLidarMapping(const std::string& map_filename);
    bool VisionStartLidarMapping();
    bool VisionStopLidarMapping(const std::string& map_filename);

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr lidar_start_sub_{nullptr};
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr lidar_stop_sub_{nullptr};
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr vision_start_sub_{nullptr};
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr vision_stop_sub_{nullptr};

    std::shared_ptr<nav2_util::ServiceClient<std_srvs::srv::SetBool>> mapping_start_client_ {nullptr};
    std::shared_ptr<nav2_util::ServiceClient<visualization::srv::Stop>> mapping_stop_client_ {nullptr};

    std::unordered_map<std::string, std::shared_ptr<nav2_util::LifecycleServiceClient>> lifecycle_nodes_;

    std::string defualt_map_filename_;
    std::string defualt_slam_type_;

    std::mutex service_mutex_;
    bool mapping_start_{false};
    bool mapping_stop_{false};
};

}  // namespace nav2_control_demo
}  // namespace cyberdog

#endif  // CYBERDOG_NAV2_CONTROL_DEMO_NAV2_MAPPING_HPP_