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

#ifndef ALGORITHM_MANAGER__GLOBAL_POSE_PUBLISHER_HPP_
#define ALGORITHM_MANAGER__GLOBAL_POSE_PUBLISHER_HPP_

#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/macros.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/create_timer_ros.h"
#include "tf2_ros/transform_listener.h"

namespace cyberdog
{
namespace algorithm
{

class PosePublisher
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(PosePublisher)

  explicit PosePublisher(rclcpp::Node * node);
  ~PosePublisher();

  void Start();
  void Stop();

  bool IsStart();
  bool IsStop();

private:
  void HandleGloablPoseCallback();

  std::string global_pose_topic_;
  rclcpp::TimerBase::SharedPtr timer_ {nullptr};

  // publisher
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pos_pub_ {nullptr};

  // tf
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // flags
  bool start_ {false};
  bool stop_ {false};
};

}  // namespace algorithm
}  // namespace cyberdog
#endif  // ALGORITHM_MANAGER__GLOBAL_POSE_PUBLISHER_HPP_
