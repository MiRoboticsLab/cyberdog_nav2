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

#ifndef CYBERDOG_NAV2_CONTROL_DEMO__NAV2_NAV2_NAVIGATTE_POSE_HPP_
#define CYBERDOG_NAV2_CONTROL_DEMO__NAV2_NAV2_NAVIGATTE_POSE_HPP_

#include <memory>
#include <vector>
#include <unordered_map>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "nav2_msgs/action/navigate_to_pose.hpp"

namespace cyberdog
{
namespace nav2_control_demo
{

class NavigattePose : public rclcpp::Node
{
public:
    using NavigationGoalHandle = rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>;

    NavigattePose();
    ~NavigattePose();

private:
    void HandleStartGoalCallback(const geometry_msgs::msg::Pose::SharedPtr msg);

    void HandleCancelGoalCallback(const std_msgs::msg::Bool::SharedPtr msg);

    void HandleGoalResponseCallback(NavigationGoalHandle::SharedPtr goal_handle);

    void HandleFeedbackCallback(NavigationGoalHandle::SharedPtr, 
        const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> feedback);

    void HandleResultCallback(const NavigationGoalHandle::WrappedResult result);

    bool SendGoal(const geometry_msgs::msg::Pose & pose);

    bool CancelGoal();

    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr send_goal_sub_{nullptr};
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr cancel_goal_sub_{nullptr};

    // nav client as request
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr action_client_ {nullptr};

    // navigation goal handle
    NavigationGoalHandle::SharedPtr nav_goal_handle_ {nullptr};

    std::mutex cancel_goal_mutex_;
    std::condition_variable cancel_goal_cv_;
    bool cancel_goal_result_{true};
};

}  // namespace nav2_control_demo
}  // namespace cyberdog

#endif  // CYBERDOG_NAV2_CONTROL_DEMO__NAV2_NAV2_NAVIGATTE_POSE_HPP_