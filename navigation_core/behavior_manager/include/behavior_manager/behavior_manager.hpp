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

#ifndef BEHAVIOR_MANAGER__BEHAVIOR_MANAGER_HPP_
#define BEHAVIOR_MANAGER__BEHAVIOR_MANAGER_HPP_

#include <memory>
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

class BehaviorManager : public rclcpp::Node
{
public:
  enum class Stage : uint8_t
  {
    kNormallyTracking,
    kAutonomouslyTracking,
    kStairJumping,
  };
  BehaviorManager();
  ~BehaviorManager();
  void Tick();
private:
  void HandleStairDetectionCallback(const std_msgs::msg::Int8::SharedPtr msg);
  void HandleStairAlginStatusCallback(const std_msgs::msg::Bool::SharedPtr msg);
  void HandleTargetPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void DecideBehaviorMode();
  bool CheckTargetStatic();
  void DoAutonomouslyTracking();
  void DoStairJumping();
  rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr stair_detected_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr stair_align_finished_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_sub_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr stair_jump_client_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr tracking_switch_client_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr autonomously_tracking_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr stair_align_trigger_client_;
  rclcpp::Client<protocol::srv::MotionResultCmd>::SharedPtr motion_jump_client_;
  bool stair_detected_{false};
  Stage stage_;
};  // class behavior_manager
}  // namespace algorithm
}  // namespace cyberdog
#endif  // BEHAVIOR_MANAGER__BEHAVIOR_MANAGER_HPP_
