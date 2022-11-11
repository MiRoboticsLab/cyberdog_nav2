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
#include <deque>
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
  explicit ModeDetector(const rclcpp::Node::SharedPtr node)
  : node_(node)
  {
    callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    rclcpp::SubscriptionOptions option;
    option.callback_group = callback_group_;
    stair_detected_sub_ = node_->create_subscription<std_msgs::msg::Int8>(
      "elevation_mapping/stair_detected",
      rclcpp::SystemDefaultsQoS(),
      std::bind(
        &ModeDetector::HandleStairDetectionCallback,
        this, std::placeholders::_1),
      option);
    target_pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
      "tracking_pose",
      rclcpp::SystemDefaultsQoS(),
      std::bind(
        &ModeDetector::HandleTargetPoseCallback,
        this, std::placeholders::_1),
      option);
    // std::thread{[this] {rclcpp::spin(node_);}}.detach();
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
  void Launch(bool stair_detect = true, bool static_detect = false)
  {
    stair_detect_ = stair_detect;
    static_detect_ = static_detect;
  }
  void Reset()
  {
    last_static_ = false;
    stair_detection_ = static_cast<int8_t>(StairDetection::kNothing);
    // TODO(lijian): 目标静止检测相关的变量复位
    // target_first_get = false;
    pose_queue_.clear();
    first_pop_ = false;
  }

private:
  void HandleStairDetectionCallback(const std_msgs::msg::Int8::SharedPtr msg)
  {
    if (!stair_detect_) {
      return;
    }
    INFO_EXPRESSION(msg->data != stair_detection_, "Detect stair: %d", msg->data);
    stair_detection_ = msg->data;
    if (stair_detection_ == static_cast<int8_t>(StairDetection::kUpStair)) {
      do_stair_jump_func_(true);
    } else if (stair_detection_ == static_cast<int8_t>(StairDetection::kDownStair)) {
      do_stair_jump_func_(false);
    }
  }
  void HandleTargetPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    if (!static_detect_) {
      return;
    }
    bool target_static = CheckTargetStatic(msg);
    INFO("%s", target_static ? "Static" : "Moving");
    if (target_static == last_static_) {
      return;
    }
    last_static_ = target_static;
    if (target_static) {
      INFO("Target Checked Static");
      do_auto_tracking_func_();
    } else {
      INFO("Target Checked Moving");
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
    if (!EnQueue(msg)) 
    {
      INFO("Pose not enough");
      return false;
    }
    auto max_x = *std::max_element(pose_x_.begin(), pose_x_.end());
    auto min_x = *std::min_element(pose_x_.begin(), pose_x_.end());
    auto diff_x = max_x - min_x;
    auto max_y = *std::max_element(pose_y_.begin(), pose_y_.end());
    auto min_y = *std::min_element(pose_y_.begin(), pose_y_.end());
    auto diff_y = max_y - min_y;

    INFO("diff_x: %f, diff_y: %f", diff_x, diff_y);
    if (diff_x > 0.3 || diff_y > 0.3) {
      return false;
    } else {
      return true;
    }
  }
  bool EnQueue(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    timestamp_.push_back(msg->header.stamp.sec);
    pose_x_.push_back(msg->pose.position.x);
    pose_y_.push_back(msg->pose.position.y);
    // INFO("back: %d, front: %d", timestamp_.back(), timestamp_.front());
    if (timestamp_.back() - timestamp_.front() < 15 && !first_pop_) {
      return false;
    }
    first_pop_ = true;
    timestamp_.pop_front();
    pose_x_.pop_front();
    pose_y_.pop_front();
    return true;
  }
  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr stair_detected_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_sub_;
  geometry_msgs::msg::PoseStamped::SharedPtr current_pose_;
  Stage stage_;
  std::function<void(bool)> do_stair_jump_func_;
  std::function<void()> do_auto_tracking_func_;
  std::function<void(bool)> do_normal_tracking_func_;
  std::deque<geometry_msgs::msg::PoseStamped> pose_queue_;
  std::deque<int> timestamp_;
  std::deque<double> pose_x_, pose_y_;
  geometry_msgs::msg::PoseStamped target_first;
  geometry_msgs::msg::PoseStamped target_current;
  double target_first_timestamp = -1;
  double target_current_timestamp = -1;
  int8_t stair_detection_{0};
  bool stair_detect_{false}, static_detect_{false};
  bool last_static_{false};
  // bool target_first_get = false;
  bool first_pop_{false};

};  // class ModeDetector
}  // namespace algorithm
}  // namespace cyberdog
#endif  // BEHAVIOR_MANAGER__MODE_DETECTOR_HPP_
