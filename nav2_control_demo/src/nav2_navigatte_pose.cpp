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

#include <chrono>

using namespace std::chrono_literals;

namespace cyberdog
{
namespace nav2_control_demo
{

NavigattePose::NavigattePose() : Node("nav2_navigatte_pose_node")
{
    send_goal_sub_ = this->create_subscription<geometry_msgs::msg::Pose>("send_goal", 10,
        std::bind(&NavigattePose::HandleStartGoalCallback, this, std::placeholders::_1));

    cancel_goal_sub_ = this->create_subscription<std_msgs::msg::Bool>("cancel_goal", 10,
        std::bind(&NavigattePose::HandleCancelGoalCallback, this, std::placeholders::_1));

    action_client_ =
        rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(this, "navigate_to_pose");
}

NavigattePose::~NavigattePose()
{
}

void NavigattePose::HandleStartGoalCallback(const geometry_msgs::msg::Pose::SharedPtr msg)
{
    if (msg == nullptr) {
        return;
    }

    bool ret = SendGoal(*msg);
    if (!ret) {
        RCLCPP_ERROR(this->get_logger(), "Send goal error.");
    } else {
        RCLCPP_INFO(this->get_logger(), "Send goal success.");
    }
}

void NavigattePose::HandleCancelGoalCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
    if (msg == nullptr) {
        return;
    }

    if (!msg->data) {
        return;
    }

    bool ret = CancelGoal();
    if (!ret) {
        RCLCPP_ERROR(this->get_logger(), "Cancel goal error.");
    } else {
        RCLCPP_INFO(this->get_logger(), "Cancel goal success.");
    }
}

bool NavigattePose::SendGoal(const geometry_msgs::msg::Pose & pose)
{
    auto send_goal_options =
        rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();

    send_goal_options.goal_response_callback = std::bind(
        &NavigattePose::HandleGoalResponseCallback, this, std::placeholders::_1); 
            
    send_goal_options.feedback_callback =
        std::bind(&NavigattePose::HandleFeedbackCallback,
            this, std::placeholders::_1, std::placeholders::_2);
    
    send_goal_options.result_callback = std::bind(
        &NavigattePose::HandleResultCallback, this, std::placeholders::_1);


    nav2_msgs::action::NavigateToPose::Goal goal;

    // Normalize the goal pose
    goal.pose.pose = pose;
    goal.pose.header.frame_id = "map";
    goal.pose.header.stamp = this->get_clock()->now();

    auto future_goal_handle = action_client_->async_send_goal(goal, send_goal_options);

    std::chrono::seconds timeout{10};
    if (future_goal_handle.wait_for(timeout) != std::future_status::ready) {
        RCLCPP_ERROR(this->get_logger(), "Wait navigation server timeout.");
        return false;
    }

    nav_goal_handle_ = future_goal_handle.get();
    if (!nav_goal_handle_) {
        RCLCPP_ERROR(this->get_logger(), "Navigation AB Goal was rejected by server");
        return false;
    }
    return true;
}

bool NavigattePose::CancelGoal()
{
    if (!nav_goal_handle_) {
        RCLCPP_WARN(this->get_logger(), "Cancel the goal faiiled, because goal handle is null");
        return false;
    }

    auto server_ready = action_client_->wait_for_action_server(std::chrono::seconds(5));
    if (!server_ready) {
        RCLCPP_ERROR(this->get_logger(),"Navigation action server(navigate_to_pose) is not available.");
        return false;
    }

    try {
        // 判断future_cancel的结果和等待
        std::unique_lock<std::mutex> lock(cancel_goal_mutex_);
        RCLCPP_INFO(this->get_logger(), "CancelGoal: run async_cancel_goal request");
        auto future_cancel = action_client_->async_cancel_goal(nav_goal_handle_);
        if (cancel_goal_cv_.wait_for(lock, 10s) == std::cv_status::timeout) {
            RCLCPP_INFO(this->get_logger(),"Get cancel_goal_cv_ value: false");
            cancel_goal_result_ = false;
        } else {
            cancel_goal_result_ = true;
            RCLCPP_INFO(this->get_logger(),"Get cancel_goal_cv_ value: true");
        }
    } catch (const std::exception & e) {
        RCLCPP_ERROR(this->get_logger(),"%s", e.what());
    }
    return cancel_goal_result_;
}

void NavigattePose::HandleGoalResponseCallback(NavigationGoalHandle::SharedPtr goal_handle)
{
    (void)goal_handle;
    RCLCPP_INFO(this->get_logger(), "Navigation AB Goal accepted");
}

void NavigattePose::HandleFeedbackCallback(NavigationGoalHandle::SharedPtr, 
    const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> feedback)

{
    (void)feedback;
}

void NavigattePose::HandleResultCallback(const NavigationGoalHandle::WrappedResult result)
{
    switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(this->get_logger(), "Navigation AB point have arrived target goal success");
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(this->get_logger(), "Navigation AB run target goal aborted");
            break;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_ERROR(this->get_logger(), "Navigation AB run target goal canceled");
            cancel_goal_cv_.notify_one();
            break;
        default:
            RCLCPP_ERROR(this->get_logger(), "Navigation AB run target goal unknown result code");
            break;
    }

}

}  // namespace nav2_control_demo
}  // namespace cyberdog

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<cyberdog::nav2_control_demo::NavigattePose>();
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();

    return 0;
}
