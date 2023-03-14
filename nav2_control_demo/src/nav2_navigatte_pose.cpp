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

#include "nav2_control_demo/nav2_navigatte_pose.hpp"

namespace cyberdog
{
namespace nav2_control_demo
{

NavigattePose::NavigattePose() : Node("navigatte_pose_node")
{
    action_client_ =
        rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(this, "navigate_to_pose");
}

NavigattePose::~NavigattePose()
{
}

void NavigattePose::HandleGoalCallback(const geometry_msgs::msg::Pose::SharedPtr msg)
{
    if (msg == nullptr) {
        return;
    }
}

}  // namespace nav2_control_demo
}  // namespace cyberdog
