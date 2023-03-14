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
    void HandleStartMappingCallback(
        const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        const std::shared_ptr<std_srvs::srv::SetBool::Response> response);

    void HandleStopMappingCallback(
        const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        const std::shared_ptr<std_srvs::srv::SetBool::Response> response);

    enum class SLAMType {
        Unknown,
        Lidar,
        Vision
    };

    bool Start(const SLAMType & type);
    bool Stop(const SLAMType & type);

    bool StartLidar();
    bool StartVision();

    bool StoptLidar();
    bool StopVision();

    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr stop_server_ {nullptr};
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr start_server_ {nullptr};

    std::shared_ptr<nav2_util::ServiceClient<std_srvs::srv::SetBool>> start_client_ {nullptr};
    std::shared_ptr<nav2_util::ServiceClient<std_srvs::srv::SetBool>> stop_client_ {nullptr};

    std::string defualt_slam_type_;

    std::mutex service_mutex_;
    bool localization_finished_start_{false};
    bool localization_finished_stop_{false};
};

}  // namespace nav2_control_demo
}  // namespace cyberdog

#endif  // CYBERDOG_NAV2_CONTROL_DEMO_NAV2_LOCALIZATION_HPP_