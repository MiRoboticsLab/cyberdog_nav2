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
  explicit ExecutorStairJumping(const std::string & node_name)
  {
    node_ = std::make_shared<rclcpp::Node>(node_name);
    stair_align_status_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
      "stair_align_finished_flag", 
      rclcpp::SystemDefaultsQoS(),
      std::bind(&ExecutorStairJumping::HandleStairAlginStatusCallback,
        this, std::placeholders::_1));
    std::thread{[this]{rclcpp::spin(node_);}}.detach();
  }
  ~ExecutorStairJumping(){}
  void Execute(bool trigger)
  {
    INFO("222");
    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = trigger;
    jump_mode_ = trigger ? StairDetection::kUpStair : StairDetection::kDownStair;
    auto callback = [this](rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future){
      if(future.get()->success){
        jumping_status_ = JumpingStatus::kAligning;
      } else {
        jumping_status_ = JumpingStatus::kAbnorm;
        handle_abnorm_func_();
      }
    };
    stair_align_trigger_client_->async_send_request(request, callback);
    INFO("333");

  }
  bool Init(
    std::function<void()> handle_jumped_func,
    std::function<void()> handle_abnorm_func)
  {
    handle_jumped_func_ = handle_jumped_func;
    handle_abnorm_func_ = handle_abnorm_func;
    return true;
  }
  JumpingStatus &
  GetStatus(){return jumping_status_;}
private:
  void HandleStairAlginStatusCallback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (msg->data == true) {
      return;
    }
    static auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    if (jump_mode_ == StairDetection::kUpStair ) {
      request->data = true;
    } else if (jump_mode_ == StairDetection::kDownStair) {
      request->data = false;
    }
    stair_jump_condition_client_->async_send_request(
      request,
      std::bind(&ExecutorStairJumping::CheckStairJumpCondition, 
        this, std::placeholders::_1));
  }
  bool CheckStairJumpCondition(rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future)
  {
    if(future.get()->success){
      jumping_status_ = JumpingStatus::kJumping;
      auto request = std::make_shared<protocol::srv::MotionResultCmd::Request>();
      if (jump_mode_ == StairDetection::kUpStair ) {
        request->motion_id = protocol::msg::MotionID::JUMP_STAIR;
      } else if (jump_mode_ == StairDetection::kDownStair) {
        // TODO(Harvey): 下台阶的动作
        // request->motion_id = protocol::msg::MotionID::JUMP_STAIR;
      }
      motion_jump_client_->async_send_request(
        request,
        std::bind(&ExecutorStairJumping::CheckStairJumpResult,
          this, std::placeholders::_1));
    } else {
      jumping_status_ = JumpingStatus::kAbnorm;
      handle_abnorm_func_();
    }
    return true;
  }
  bool CheckStairJumpResult(rclcpp::Client<protocol::srv::MotionResultCmd>::SharedFuture future)
  {
    if(future.get()->code){
      jumping_status_ = JumpingStatus::kJumped;
      handle_jumped_func_();
    } else {
      jumping_status_ = JumpingStatus::kAbnorm;
      handle_abnorm_func_();
    }
    return true;
  }
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr stair_align_status_sub_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr stair_jump_condition_client_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr stair_align_trigger_client_;
  rclcpp::Client<protocol::srv::MotionResultCmd>::SharedPtr motion_jump_client_;
  std::function<void()> handle_abnorm_func_;
  std::function<void()> handle_jumped_func_;
  // Stage stage_;
  JumpingStatus jumping_status_;
  int8_t stair_detection_{0};
  bool stair_aligned_{false};
  StairDetection jump_mode_{StairDetection::kNothing};
};  // class ExecutorStairJumping
}  // namespace algorithm
}  // namespace cyberdog
#endif  // BEHAVIOR_MANAGER__EXECUTOR_STAIR_JUMPING_HPP_
