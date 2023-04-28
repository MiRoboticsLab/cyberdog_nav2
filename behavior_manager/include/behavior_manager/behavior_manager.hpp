// Copyright (c) 2023 Beijing Xiaomi Mobile Software Co., Ltd. All rights reserved.
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
#include <string>
#include <unordered_map>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int8.hpp"
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "protocol/srv/motion_result_cmd.hpp"
#include "protocol/srv/audio_text_play.hpp"
#include "protocol/msg/audio_play.hpp"
#include "cyberdog_debug/backtrace.hpp"
#include "behavior_manager/mode_detector.hpp"
#include "behavior_manager/executor_auto_tracking.hpp"
#include "behavior_manager/executor_stair_jumping.hpp"
namespace cyberdog
{
namespace algorithm
{

class BehaviorManager
{
public:
  enum class BehaviorStatus : uint8_t
  {
    kNormTracking,
    kAutoTracking,
    kStairJumping,
    kAbnorm
  };
  explicit BehaviorManager(const std::string & node_name)
  {
    node_ = std::make_shared<rclcpp::Node>(node_name);
    callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    mode_detector_ = std::make_shared<ModeDetector>(node_);
    mode_detector_->Init(
      std::bind(&BehaviorManager::DoStairJump, this, std::placeholders::_1),
      std::bind(&BehaviorManager::DoAutoTracking, this),
      std::bind(&BehaviorManager::DoNormallyTracking, this, std::placeholders::_1)
    );
    executor_auto_tracking_ = std::make_shared<ExecutorAutoTracking>(node_);
    executor_stair_jumping_ = std::make_shared<ExecutorStairJumping>(node_);
    executor_stair_jumping_->Init(
      std::bind(&BehaviorManager::HandleJumped, this),
      std::bind(&BehaviorManager::HandleJumpFailed, this));
    tracking_switch_client_ = node_->create_client<std_srvs::srv::SetBool>(
      "tracking_command",
      rmw_qos_profile_services_default,
      callback_group_);
    audio_play_client_ = node_->create_client<protocol::srv::AudioTextPlay>(
      "speech_text_play",
      rmw_qos_profile_services_default,
      callback_group_);
    status_map_.emplace(BehaviorStatus::kAutoTracking, "AutoTracking");
    status_map_.emplace(BehaviorStatus::kNormTracking, "NormTracking");
    status_map_.emplace(BehaviorStatus::kStairJumping, "StairJumping");
    status_map_.emplace(BehaviorStatus::kAbnorm, "Abnorm");
    ros_executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    ros_executor_->add_node(node_);
    std::thread{[this]() {ros_executor_->spin();}}.detach();
  }
  void RegisterStateCallback(std::function<void(BehaviorStatus)> state_callback)
  {
    state_callback_ = state_callback;
  }
  void Launch(bool stair_detect, bool static_detect)
  {
    mode_detector_->Launch(stair_detect, static_detect);
  }
  void Reset()
  {
    executor_stair_jumping_->Interupt();
    executor_auto_tracking_->Cancel();
    mode_detector_->Reset();
    DoNormallyTracking(true);
    status_ = BehaviorStatus::kNormTracking;
    state_callback_(status_);
  }
  ~BehaviorManager() {}

private:
  bool CheckStatusValid()
  {
    return status_ == BehaviorStatus::kNormTracking;
  }
  void DoAutoTracking()
  {
    if (!CheckStatusValid()) {
      WARN("Cannot DoAutoTracking when %s", status_map_.at(status_).c_str());
      return;
    }
    DoNormallyTracking(false);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    executor_auto_tracking_->Execute(true);
  }
  void DoStairJump(bool trigger)
  {
    if (!CheckStatusValid()) {
      // ERROR(
      //   "Cannot do %s jump when %s", trigger ? "upstair" : "downstair", status_map_.at(
      //     status_).c_str());
      return;
    }
    if (!DoNormallyTracking(false)) {
      ERROR("Cannot Pause NormTracking");
      return;
    }
    executor_stair_jumping_->Execute(trigger);
    status_ = BehaviorStatus::kStairJumping;
    state_callback_(status_);
  }
  bool DoNormallyTracking(bool trigger)
  {
    if (trigger) {
      executor_auto_tracking_->Interupt();
    }
    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = trigger;
    auto future = tracking_switch_client_->async_send_request(request);
    if (future.wait_for(std::chrono::milliseconds(2000)) == std::future_status::timeout) {
      ERROR("Cannot get result of %s NormTracking", trigger ? "resume" : "pause");
      return false;
    }
    return future.get()->success;
  }
  bool HandleJumped()
  {
    status_ = BehaviorStatus::kNormTracking;
    state_callback_(status_);
    return DoNormallyTracking(true);
  }
  bool HandleJumpFailed()
  {
    status_ = BehaviorStatus::kAbnorm;
    state_callback_(status_);
    HandleAbnormStatus();
    return true;
  }
  void HandleAbnormStatus()
  {
    INFO("Abnorm");
    auto request = std::make_shared<protocol::srv::AudioTextPlay::Request>();
    request->module_name = node_->get_name();
    request->is_online = true;
    request->text = "跳台阶出现异常，跳台阶出现异常";
    auto callback = [](rclcpp::Client<protocol::srv::AudioTextPlay>::SharedFuture future) {
        INFO("Audio play result: %s", future.get()->status == 0 ? "success" : "failed");
      };
    auto future_online = audio_play_client_->async_send_request(request, callback);
    if (future_online.wait_for(std::chrono::milliseconds(2000)) == std::future_status::timeout) {
      ERROR("Cannot get response of AudioPlay");
    }
    request->is_online = false;
    request->speech.play_id = protocol::msg::AudioPlay::PID_FACE_ENTRY_FIX_STABLE;
    auto future_offline = audio_play_client_->async_send_request(request, callback);
    if (future_offline.wait_for(std::chrono::milliseconds(2000)) == std::future_status::timeout) {
      ERROR("Cannot get response of AudioPlay");
    }
  }
  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr tracking_switch_client_;
  rclcpp::Client<protocol::srv::AudioTextPlay>::SharedPtr audio_play_client_;
  ModeDetector::Stage stage_working_, stage_detected_;
  std::shared_ptr<ModeDetector> mode_detector_;
  std::shared_ptr<ExecutorAutoTracking> executor_auto_tracking_;
  std::shared_ptr<ExecutorStairJumping> executor_stair_jumping_;
  rclcpp::Executor::SharedPtr ros_executor_;
  BehaviorStatus status_{BehaviorStatus::kNormTracking};
  std::unordered_map<BehaviorStatus, std::string> status_map_;
  std::function<void(BehaviorStatus)> state_callback_;
  bool stair_detected_{false}, stair_aligned_{false}, stair_align_timeout_{false};
  bool stair_possible_jump_{false};
};  // class BehaviorManager
}  // namespace algorithm
}  // namespace cyberdog
#endif  // BEHAVIOR_MANAGER__BEHAVIOR_MANAGER_HPP_
