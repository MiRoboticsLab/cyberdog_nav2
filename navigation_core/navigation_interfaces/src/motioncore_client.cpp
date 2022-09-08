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

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("client");
  auto client = rclcpp_action::create_client<protocol::action::Navigation>(
    node,
    "CyberdogNavigation");
  protocol::action::Navigation_Goal goal;
  std::cout << argv[1] << std::endl;
  if (std::string(argv[1]) == std::string("1")) { // 开始AB
    std::cout << "Start AB" << std::endl;
    goal.nav_type = protocol::action::Navigation_Goal::NAVIGATION_TYPE_START_AB;
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.pose.orientation.w = 1;
    pose.pose.position.x = 0.5;
    goal.poses.push_back(pose);
    client->async_send_goal(goal);
  } else if (std::string(argv[1]) == std::string("2")) {  // 结束AB
    std::cout << "Stop AB" << std::endl;

  } else if (std::string(argv[1]) == std::string("6")) { // 开始建图
    std::cout << "Start Mapping" << std::endl;
    goal.nav_type = protocol::action::Navigation_Goal::NAVIGATION_TYPE_START_MAPPING;
    client->async_send_goal(goal);
  } else if (std::string(argv[1]) == std::string("7")) { // 结束建图
    std::cout << "Stop Mapping" << std::endl;
    goal.nav_type = protocol::action::Navigation_Goal::NAVIGATION_TYPE_STOP_MAPPING;
  }
  rclcpp::shutdown();
  return 0;
}
