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

#ifndef BEHAVIOR_MANAGER__MODE_DETECTOR_HPP_
#define BEHAVIOR_MANAGER__MODE_DETECTOR_HPP_

#include <memory>
#include <string>
#include <unordered_map>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int8.hpp"
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "protocol/srv/motion_result_cmd.hpp"
#include "cyberdog_debug/backtrace.hpp"
namespace cyberdog
{
namespace algorithm
{
class ModeDetector
{
public:
  enum class Stage : uint8_t
  {
    kNormallyTracking,
    kAutonomouslyTracking,
    kStairJumping,
    kAbnorm,
  };
  enum class StairDetection : int8_t
  {
    kUpStair = 1,
    kNothing = 0,
    kDownStair = -1,
  };
  explicit ModeDetector(const std::string & node_name)
  {
    node_ = std::make_shared<rclcpp::Node>(node_name);
    stair_detected_sub_ = node_->create_subscription<std_msgs::msg::Int8>(
      "elevation_mapping/stair_detected",
      rclcpp::SystemDefaultsQoS(),
      std::bind(
        &ModeDetector::HandleStairDetectionCallback,
        this, std::placeholders::_1));
    target_pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
      "tracking_pose",
      rclcpp::SystemDefaultsQoS(),
      std::bind(
        &ModeDetector::HandleTargetPoseCallback,
        this, std::placeholders::_1));
    std::thread{[this] {rclcpp::spin(node_);}}.detach();
  }
  ~ModeDetector() {}
  bool Init(
    std::function<void(bool)> do_stair_jump_func,
    std::function<void()> do_auto_tracking_func,
    std::function<void(bool)> do_normal_tracking_func
  )
  {
    do_stair_jump_func_ = do_stair_jump_func;
    do_auto_tracking_func_ = do_auto_tracking_func;
    do_normal_tracking_func_ = do_normal_tracking_func;
    return true;
  }

private:
  void HandleStairDetectionCallback(const std_msgs::msg::Int8::SharedPtr msg)
  {
    INFO_EXPRESSION(msg->data != 0, "Detect stair: %d", msg->data);
    stair_detection_ = msg->data;
    if (stair_detection_ == static_cast<int8_t>(StairDetection::kUpStair)) {
      do_stair_jump_func_(true);
    } else if (stair_detection_ == static_cast<int8_t>(StairDetection::kDownStair)) {
      do_stair_jump_func_(false);
    }
  }
  void HandleTargetPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    if (!CheckTargetStatic(msg)) {
      do_auto_tracking_func_();
    } else {
      do_normal_tracking_func_(true);
    }
  }
  /**
   * @brief
   * 检测目标是否处于静止状态，判断依据：在设定时长内目标的位姿没有设定范围外的变化
   *
   * @param msg
   * @return true
   * @return false
   */
  bool CheckTargetStatic(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    (void)msg;
    return true;
  }
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr stair_detected_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_sub_;
  geometry_msgs::msg::PoseStamped::SharedPtr current_pose_;
  Stage stage_;
  std::function<void(bool)> do_stair_jump_func_;
  std::function<void()> do_auto_tracking_func_;
  std::function<void(bool)> do_normal_tracking_func_;
  int8_t stair_detection_{0};
};  // class ModeDetector
}  // namespace algorithm
}  // namespace cyberdog
#endif  // BEHAVIOR_MANAGER__MODE_DETECTOR_HPP_
