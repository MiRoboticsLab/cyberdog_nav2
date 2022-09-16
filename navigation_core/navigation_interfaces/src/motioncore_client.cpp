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
#include <time.h>

#include <algorithm>
#include <memory>
#include <queue>
#include <string>
#include <unordered_set>
#include <rclcpp_action/client.hpp>
#include <protocol/action/navigation.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include "motion_core/motion_core_node.hpp"

#include <iostream>
#include <csignal>
#include <unistd.h>

// std::shared_future<std::shared_ptr<rclcpp_action::ClientGoalHandle<protocol::action::Navigation>>> goal_handle_future

rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("client");
auto client = rclcpp_action::create_client<protocol::action::Navigation>(
  node, "CyberdogNavigation");

std::shared_ptr<rclcpp_action::ClientGoalHandle<protocol::action::Navigation>> goal_handle;


void feedback_callback(
  rclcpp_action::ClientGoalHandle<protocol::action::Navigation>::SharedPtr,
  const std::shared_ptr<const protocol::action::Navigation_Feedback> feedback)
{
  std::cout << feedback->status << std::endl;
}


void signalHandler( int signum )
{
  std::cout << "Interrupt signal (" << signum << ") received.\n";

    // 清理并关闭
    // 终止程序  
  client->async_cancel_goal(goal_handle);

  exit(signum);  

}

int main(int argc, char ** argv)
{
  // signal(SIGINT, signalHandler);  
  rclcpp::init(argc, argv);

  if(!client->wait_for_action_server(std::chrono::seconds(1))) {
    ERROR("ActionServer not avilable");
  }
  protocol::action::Navigation_Goal goal;

  std::cout << argv[1] << std::endl;

  goal.nav_type = std::stoi(std::string(argv[1]));
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "map";
  pose.pose.orientation.w = 1;
  pose.pose.position.x = 0.5;
  goal.poses.push_back(pose);
  
  auto send_goal_option = rclcpp_action::Client<protocol::action::Navigation>::SendGoalOptions();
  send_goal_option.feedback_callback = feedback_callback;
  auto goal_handle_future = client->async_send_goal(goal, send_goal_option);
  if (rclcpp::spin_until_future_complete(node, goal_handle_future) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node->get_logger(), "send goal call failed :(");
    return 1;
  }
  // rclcpp_action::ClientGoalHandle<Fibonacci>::SharedPtr goal_handle = goal_handle_future.get();
  goal_handle = goal_handle_future.get();
  
  if (!goal_handle) {
    RCLCPP_ERROR(node->get_logger(), "Goal was rejected by server");
    return 1;
  }

  // Wait for the server to be done with the goal
  auto result_future = client->async_get_result(goal_handle);

  RCLCPP_INFO(node->get_logger(), "Waiting for result");
  if (rclcpp::spin_until_future_complete(node, result_future) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node->get_logger(), "get result call failed :(");
    return 1;
  }

  // rclcpp_action::ClientGoalHandle<Fibonacci>::WrappedResult wrapped_result = result_future.get();
  auto wrapped_result = result_future.get();

  switch (wrapped_result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_ERROR(node->get_logger(), "Goal was succeed");
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(node->get_logger(), "Goal was aborted");
      return 1;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(node->get_logger(), "Goal was canceled");
      return 1;
    default:
      RCLCPP_ERROR(node->get_logger(), "Unknown result code");
      return 1;
  }


  rclcpp::shutdown();
  return 0;
}
