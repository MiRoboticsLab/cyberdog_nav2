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

BehaviorManager::BehaviorManager(const std::string & node_name)
: rclcpp::Node(node_name)
{
  // stair_detected_sub_ = this->create_subscription<std_msgs::msg::Int8>(
  //   "elevation_mapping/stair_detected",
  //   rclcpp::SystemDefaultsQoS(), 
  //   std::bind(&BehaviorManager::HandleStairDetectionCallback, this, std::placeholders::_1));
  // stair_align_finished_sub_ = this->create_subscription<std_msgs::msg::Bool>(
  //   "stair_align_finished_flag",
  //   rclcpp::SystemDefaultsQoS(),
  //   std::bind(&BehaviorManager::HandleStairAlginStatusCallback, this, std::placeholders::_1));
  // target_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
  //   "tracking_pose",
  //   rclcpp::SystemDefaultsQoS(),
  //   std::bind(&BehaviorManager::HandleTargetPoseCallback, this, std::placeholders::_1));
  // stair_jump_client_ = this->create_client<std_srvs::srv::SetBool>("elevation_mapping/stair_jump_detected");
  // stair_align_trigger_client_ = this->create_client<std_srvs::srv::Trigger>("stair_align");
  // motion_jump_client_ = this->create_client<protocol::srv::MotionResultCmd>("result_motion_command");
  tracking_switch_client_ = this->create_client<std_srvs::srv::SetBool>("tracking_command");
  // autonomously_tracking_client_ = this->create_client<std_srvs::srv::SetBool>("");
}

BehaviorManager::~BehaviorManager()
{
}

bool BehaviorManager::CheckTargetStatic()
{

}

void BehaviorManager::DecideBehaviorMode()
{
  while (rclcpp::ok())
  {
    switch (stage_working_)
    {
      case ModeDetector::Stage::kNormallyTracking:
      {
        if (stage_detected_ == ModeDetector::Stage::kAutonomouslyTracking) {
          // 进入自主遛狗模式
          stage_working_ = ModeDetector::Stage::kAutonomouslyTracking;
        } else if (stage_detected_ == ModeDetector::Stage::kStairJumping) {
          // 进入跳台阶模式
          stage_working_ = ModeDetector::Stage::kStairJumping;
        }
      }
        break;
      
      case ModeDetector::Stage::kStairJumping:
      {
        if (executor_stair_jumping_.GetStatus() == ExecutorStairJumping::JumpingStatus::kIdle) {
          executor_stair_jumping_.Execute(true);
        } else if (executor_stair_jumping_.GetStatus() == ExecutorStairJumping::JumpingStatus::kJumped) {
          stage_working_ = ModeDetector::Stage::kNormallyTracking;
        } else if (executor_stair_jumping_.GetStatus() == ExecutorStairJumping::JumpingStatus::kAbnorm) {
          stage_working_ = ModeDetector::Stage::kAbnorm;
        }
      }
        break;

      case ModeDetector::Stage::kAutonomouslyTracking:
      {
        if (stage_detected_ == ModeDetector::Stage::kNormallyTracking) {
          stage_working_ = ModeDetector::Stage::kNormallyTracking;
          // 恢复正常跟随
          executor_auto_tracking_.Execute(false);
        }
        // 自主遛狗
        executor_auto_tracking_.Execute(true);
      }
        break;

      case ModeDetector::Stage::kAbnorm:
      {
        executor_auto_tracking_.Execute(false);
        executor_stair_jumping_.Execute(false);
      }
        break;

      default:
        break;
    }
  }
  
}
}
}
