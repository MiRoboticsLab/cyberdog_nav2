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

#ifndef BEHAVIOR_MANAGER__EXECUTOR_STAIR_JUMPING_HPP_
#define BEHAVIOR_MANAGER__EXECUTOR_STAIR_JUMPING_HPP_

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
#include "protocol/msg/motion_id.hpp"
#include "cyberdog_debug/backtrace.hpp"
namespace cyberdog
{
namespace algorithm
{

class ExecutorStairJumping
{
public:
  enum class JumpingStatus : int8_t
  {
    kUninit,
    kIdle,
    kAligning,
    kJumping,
    kJumped,
    kAbnorm,
  };
  enum class StairDetection : int8_t
  {
    kUpStair = 1,
    kNothing = 0,
    kDownStair = -1,
  };
  explicit ExecutorStairJumping(const rclcpp::Node::SharedPtr node)
  : node_(node)
  {
    // executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    // executor_->add_node(node_);
    callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    rclcpp::SubscriptionOptions option;
    option.callback_group = callback_group_;
    stair_align_status_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
      "stair_align_finished_flag",
      rclcpp::SystemDefaultsQoS(),
      std::bind(
        &ExecutorStairJumping::HandleStairAlginStatusCallback,
        this, std::placeholders::_1),
      option);
    start_stair_align_client_ = node_->create_client<std_srvs::srv::SetBool>(
      "start_stair_align",
      rmw_qos_profile_services_default,
      callback_group_);
    stop_stair_align_client_ = node_->create_client<std_srvs::srv::Trigger>(
      "stop_stair_align",
      rmw_qos_profile_services_default,
      callback_group_);
    stair_jump_condition_client_ = node_->create_client<std_srvs::srv::SetBool>(
      "elevation_mapping/stair_jump_detected",
      rmw_qos_profile_services_default,
      callback_group_);
    motion_jump_client_ = node_->create_client<protocol::srv::MotionResultCmd>(
      "motion_result_cmd",
      rmw_qos_profile_services_default,
      callback_group_);
    jump_status_map_.emplace(JumpingStatus::kUninit, "Uninit");
    jump_status_map_.emplace(JumpingStatus::kIdle, "Idle");
    jump_status_map_.emplace(JumpingStatus::kAligning, "Aligning");
    jump_status_map_.emplace(JumpingStatus::kJumping, "Jumping");
    jump_status_map_.emplace(JumpingStatus::kJumped, "Jumped");
    jump_status_map_.emplace(JumpingStatus::kAbnorm, "Abnorm");
    align_timeout_thread_ = std::thread{std::bind(&ExecutorStairJumping::CheckAlignTimeout, this)};
  }
  ~ExecutorStairJumping()
  {
    check_align_timeout_cv_.notify_one();
    align_timeout_thread_.join();
  }
  void Execute(bool trigger)
  {
    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = trigger;
    jump_mode_ = trigger ? StairDetection::kUpStair : StairDetection::kDownStair;
    auto callback = [this, trigger](rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future) {
        if (future.get()->success) {
          std::unique_lock<std::mutex> lk(this->check_align_timeout_mutex_);
          jumping_status_ = JumpingStatus::kAligning;
          this->check_align_timeout_cv_.notify_one();
        } else {
          INFO("Cannot launch %s align", trigger ? "upstair" : "downstair");
          jumping_status_ = JumpingStatus::kAbnorm;
          handle_abnorm_func_();
        }
      };
    INFO("Will launch %s align", trigger ? "upstair" : "downstair");
    auto future = start_stair_align_client_->async_send_request(request, callback);
    if (future.wait_for(std::chrono::milliseconds(2000)) == std::future_status::timeout) {
      ERROR("Cannot Get response when launching %s align", trigger ? "upstair" : "downstair");
      handle_abnorm_func_();
    }
  }
  bool Init(
    std::function<void()> handle_jumped_func,
    std::function<void()> handle_abnorm_func)
  {
    handle_jumped_func_ = handle_jumped_func;
    handle_abnorm_func_ = handle_abnorm_func;
    jumping_status_ = JumpingStatus::kIdle;
    return true;
  }
  JumpingStatus &
  GetStatus() {return jumping_status_;}

private:
  void CheckAlignTimeout()
  {
    static int unit = 500;
    static int count = 0;
    static int timeout_count = 60;
    while (rclcpp::ok()) {
      if (!check_align_timeout_start_) {
        std::unique_lock<std::mutex> lk(check_align_timeout_mutex_);
        check_align_timeout_cv_.wait(lk);
        check_align_timeout_start_ = true;
      }
      INFO("Jumping status: %s", jump_status_map_.at(jumping_status_).c_str());
      if (jumping_status_ == JumpingStatus::kAligning) {
        ++count;
      } else {
        count = 0;
        check_align_timeout_start_ = false;
      }
      if (count > timeout_count) {
        ERROR("Stair align timeout for %ds", unit * timeout_count / 1000);
        count = 0;
        check_align_timeout_start_ = false;
        auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
        auto callback = [](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
            if (!future.get()->success) {
              ERROR("Cannot stop stair align");
            }
          };
        auto future = stop_stair_align_client_->async_send_request(request, callback);
        if (future.wait_for(std::chrono::milliseconds(2000)) == std::future_status::timeout) {
          ERROR("Cannot get result of stopping stair align");
        }
        handle_abnorm_func_();
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(unit));
    }
  }
  void HandleStairAlginStatusCallback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (msg->data == false) {
      return;
    }
    if (jumping_status_ != JumpingStatus::kAligning) {
      return;
    }
    INFO("Stair Aligned: %d", (int)jump_mode_);
    jumping_status_ = JumpingStatus::kJumping;
    static auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    if (jump_mode_ == StairDetection::kUpStair) {
      request->data = true;
    } else if (jump_mode_ == StairDetection::kDownStair) {
      request->data = false;
    }
    auto future = stair_jump_condition_client_->async_send_request(
      request,
      std::bind(
        &ExecutorStairJumping::CheckStairJumpCondition,
        this, std::placeholders::_1));
    if (future.wait_for(std::chrono::milliseconds(2000)) == std::future_status::timeout) {
      ERROR("Cannot check stair jump condition");
      handle_abnorm_func_();
    }
  }
  bool CheckStairJumpCondition(rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future)
  {
    if (future.get()->success) {
      INFO("Allow to Jump: %d", (int)jump_mode_);
      jumping_status_ = JumpingStatus::kJumping;
      auto request = std::make_shared<protocol::srv::MotionResultCmd::Request>();
      if (jump_mode_ == StairDetection::kUpStair) {
        request->motion_id = protocol::msg::MotionID::JUMP_UPSTAIR;
      } else if (jump_mode_ == StairDetection::kDownStair) {
        request->motion_id = protocol::msg::MotionID::JUMP_DOWNSTAIR;
      }
      INFO("Trying to Jump: %d", (int)jump_mode_);
      auto future = motion_jump_client_->async_send_request(
        request,
        std::bind(
          &ExecutorStairJumping::CheckStairJumpResult,
          this, std::placeholders::_1));
      if (future.wait_for(std::chrono::milliseconds(10000)) == std::future_status::timeout) {
        ERROR("Cannot Get stair jump result");
        handle_abnorm_func_();
      }
    } else {
      WARN("Cannot Jump: %d", (int)jump_mode_);
      jumping_status_ = JumpingStatus::kAbnorm;
      handle_abnorm_func_();
    }
    return true;
  }
  bool CheckStairJumpResult(rclcpp::Client<protocol::srv::MotionResultCmd>::SharedFuture future)
  {
    if (future.get()->code == 0) {
      INFO("Succeed to Jump %d", (int)jump_mode_);
      jumping_status_ = JumpingStatus::kJumped;
      handle_jumped_func_();
    } else {
      INFO("Failed to Jump %d", (int)jump_mode_);
      jumping_status_ = JumpingStatus::kAbnorm;
      handle_abnorm_func_();
    }
    return true;
  }
  rclcpp::Node::SharedPtr node_;
  // rclcpp::executors::MultiThreadedExecutor::SharedPtr executor_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr stair_align_status_sub_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr stair_jump_condition_client_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr start_stair_align_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr stop_stair_align_client_;
  rclcpp::Client<protocol::srv::MotionResultCmd>::SharedPtr motion_jump_client_;
  std::function<void()> handle_abnorm_func_;
  std::function<void()> handle_jumped_func_;
  std::unordered_map<JumpingStatus, std::string> jump_status_map_;
  JumpingStatus jumping_status_{JumpingStatus::kUninit};
  std::mutex check_align_timeout_mutex_;
  std::condition_variable check_align_timeout_cv_;
  StairDetection jump_mode_{StairDetection::kNothing};
  std::thread align_timeout_thread_;
  int8_t stair_detection_{0};
  bool stair_aligned_{false};
  bool check_align_timeout_start_{false};
};  // class ExecutorStairJumping
}  // namespace algorithm
}  // namespace cyberdog
#endif  // BEHAVIOR_MANAGER__EXECUTOR_STAIR_JUMPING_HPP_
