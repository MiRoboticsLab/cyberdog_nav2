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

#include <memory>
#include <vector>
#include <string>
#include "behavior_manager/behavior_manager.hpp"
#include "cyberdog_common/cyberdog_log.hpp"
namespace cyberdog
{
namespace algorithm
{

BehaviorManager::BehaviorManager()
: rclcpp::Node("behavior_manager")
{
  stair_detected_sub_ = this->create_subscription<std_msgs::msg::Int8>(
    "elevation_mapping/stair_detected",
    rclcpp::SystemDefaultsQoS(), 
    std::bind(&BehaviorManager::HandleStairDetectionCallback, this, std::placeholders::_1));
  stair_align_finished_sub_ = this->create_subscription<std_msgs::msg::Bool>(
    "stair_align_finished_flag",
    rclcpp::SystemDefaultsQoS(),
    std::bind(&BehaviorManager::HandleStairAlginStatusCallback, this, std::placeholders::_1));
  target_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "tracking_pose",
    rclcpp::SystemDefaultsQoS(),
    std::bind(&BehaviorManager::HandleTargetPoseCallback, this, std::placeholders::_1));
  stair_jump_client_ = this->create_client<std_srvs::srv::SetBool>("elevation_mapping/stair_jump_detected");
  stair_align_trigger_client_ = this->create_client<std_srvs::srv::Trigger>("stair_align");
  motion_jump_client_ = this->create_client<protocol::srv::MotionResultCmd>("result_motion_command");
  tracking_switch_client_ = this->create_client<std_srvs::srv::SetBool>("tracking_command");
  autonomously_tracking_client_ = this->create_client<std_srvs::srv::SetBool>("");
}

BehaviorManager::~BehaviorManager()
{
}

bool BehaviorManager::CheckTargetStatic()
{

}

void BehaviorManager::DecideBehaviorMode()
{
  while (rclcpp::ok()) {
    if (stage_ != Stage::kNormallyTracking) {
      return;
    }
    if (CheckTargetStatic()) {
      stage_ = Stage::kAutonomouslyTracking;
      auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
      autonomously_tracking_client_->async_send_request(request);
    } else if (stair_detected_) {
      stage_ = Stage::kStairJumping;
    } else {
      stage_ = Stage::kNormallyTracking;
    }
  }
  while (rclcpp::ok())
  {
    switch (stage_)
    {
      case Stage::kNormallyTracking:
      {
        if (CheckTargetStatic()) {
          stage_ = Stage::kAutonomouslyTracking;
          auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
          autonomously_tracking_client_->async_send_request(request);
        } else if (stair_detected_) {
          stage_ = Stage::kStairJumping;
        }
      }
        break;
      
      case Stage::kStairJumping:
        break;

      case Stage::kAutonomouslyTracking:

      default:
        break;
    }
  }
  
}

int main(int argc, char ** argv)
{
  LOGGER_MAIN_INSTANCE("BehaviorManager");
  cyberdog::debug::register_signal();
  rclcpp::init(argc, argv);
  auto atm = std::make_shared<cyberdog::algorithm::BehaviorManager>();
  rclcpp::spin(atm);
}
