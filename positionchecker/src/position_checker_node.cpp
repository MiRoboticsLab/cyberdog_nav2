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

#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include <memory>
#include <string>
#include <vector>

#include "positionchecker/position_checker_node.hpp"
#include "cyberdog_common/cyberdog_log.hpp"

using namespace std::chrono_literals;
namespace CYBERDOG_NAV
{

PositionChecker::PositionChecker()
: rclcpp::Node("PositionChecker"), looping_(false)
{
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    get_node_base_interface(), get_node_timers_interface());
  tf_buffer_->setCreateTimerInterface(timer_interface);
  tf_buffer_->setUsingDedicatedThread(true);
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  enable_service = create_service<SetBool>(
    "PoseEnable",
    std::bind(
      &PositionChecker::serviceCallback, this, std::placeholders::_1,
      std::placeholders::_2, std::placeholders::_3));
  pos_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
    "dog_pose", rclcpp::SystemDefaultsQoS());
}

PositionChecker::~PositionChecker() {}

void PositionChecker::loop()
{
  geometry_msgs::msg::PoseStamped pose_based_on_global_frame;
  std::string global_frame_, robot_base_frame_;

  while (true) {
    if (!looping_) {
      return;
    }

    if (!nav2_util::getCurrentPose(
        pose_based_on_global_frame, *tf_buffer_,
        "map", "base_link"))
    {
      WARN("Failed to obtain current pose based on map coordinate system.");
      std::this_thread::sleep_for(std::chrono::seconds(5));
      continue;
    } else {
      pos_pub_->publish(pose_based_on_global_frame);
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }
}
void PositionChecker::serviceCallback(
  const std::shared_ptr<rmw_request_id_t>,
  const std::shared_ptr<SetBool::Request> request,
  std::shared_ptr<SetBool::Response> response)
{
  if (request->data == true && !looping_) {
    // if (tf_listener_ == nullptr) {
    //   tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    // }

    looping_ = true;
    INFO("Request start report robot's realtime pose.");
    loop_thread_ = std::make_shared<std::thread>(&PositionChecker::loop, this);
  } else if (request->data == false) {
    INFO("Request stop report robot's realtime pose.");
    looping_ = false;
    loop_thread_->join();
  }
  response->success = true;
}

}  // namespace CYBERDOG_NAV
