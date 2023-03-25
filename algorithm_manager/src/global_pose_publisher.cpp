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

#include <chrono>
#include <memory>

#include "nav2_util/robot_utils.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "algorithm_manager/global_pose_publisher.hpp"
#include "cyberdog_common/cyberdog_log.hpp"

using namespace std::chrono_literals;   // NOLINT

namespace cyberdog
{
namespace algorithm
{

PosePublisher::PosePublisher(const rclcpp::Node::SharedPtr node)
{
  // tf
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    node->get_node_base_interface(),
    node->get_node_timers_interface());
  tf_buffer_->setCreateTimerInterface(timer_interface);
  tf_buffer_->setUsingDedicatedThread(true);
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // pose publisher
  global_pose_topic_ = "dog_pose";
  pos_pub_ = node->create_publisher<geometry_msgs::msg::PoseStamped>(
    global_pose_topic_, rclcpp::SystemDefaultsQoS());

  // timer
  timer_ = node->create_wall_timer(
    1000ms, std::bind(&PosePublisher::HandleGloablPoseCallback, this));
}

PosePublisher::~PosePublisher()
{
}

void PosePublisher::Start()
{
  start_ = true;
  stop_ = false;
  timer_->execute_callback();
}

void PosePublisher::Close()
{
  start_ = false;
  stop_ = true;
  timer_->cancel();
}

bool PosePublisher::IsStart()
{
  return start_;
}

bool PosePublisher::IsStop()
{
  return stop_;
}

void PosePublisher::HandleGloablPoseCallback()
{
  if (start_) {
    geometry_msgs::msg::PoseStamped gloabl_pose;
    if (!nav2_util::getCurrentPose(gloabl_pose, *tf_buffer_, "map", "base_link", 2.0)) {
      WARN("Failed to obtain current pose based on map coordinate system.");
      std::this_thread::sleep_for(std::chrono::seconds(5));
      return;
    } else {
      pos_pub_->publish(gloabl_pose);
    }
  }
}

}  // namespace algorithm
}  // namespace cyberdog
