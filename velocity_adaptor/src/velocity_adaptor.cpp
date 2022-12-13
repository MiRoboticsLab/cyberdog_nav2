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

#include <vector>

#include "velocity_adaptor/velocity_adaptor.hpp"
#include "protocol/msg/motion_id.hpp"

namespace cyberdog
{
namespace navigation
{

VelocityAdaptor::VelocityAdaptor()
: Node("velocity_adaptor"), gait_motion_id(309),
  gait_shape_value(0), gait_step_height({0.06, 0.06}), 
  twist_history_duration_(rclcpp::Duration::from_seconds(0.5))
{
  motion_vel_cmd_pub_ = this->create_publisher<::protocol::msg::MotionServoCmd>(
    "motion_servo_cmd", rclcpp::SystemDefaultsQoS());

  nav_cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", rclcpp::SystemDefaultsQoS(),
    std::bind(&VelocityAdaptor::HandleNavCommandVelocity, this, std::placeholders::_1));

  change_gait_srv_ = this->create_service<MotionResultSrv>(
    "velocity_adaptor_gait",
    std::bind(
      &VelocityAdaptor::VelocityAdaptorGaitCallback, this, std::placeholders::_1,
      std::placeholders::_2));
  twist_cumulate_.linear.x = 0;
  twist_cumulate_.linear.y = 0;
  twist_cumulate_.linear.z = 0;
  twist_cumulate_.angular.x = 0;
  twist_cumulate_.angular.y = 0;
  twist_cumulate_.angular.z = 0;      
  stop_vel_occur_ = false;
}

VelocityAdaptor::~VelocityAdaptor()
{
}

void VelocityAdaptor::HandleNavCommandVelocity(geometry_msgs::msg::Twist::SharedPtr msg)
{
  if (msg == nullptr) {
    INFO("VelocityAdaptor cmd vel == nullptr.");
    return;
  }
  PublishCommandVelocity(msg);
}

void VelocityAdaptor::VelocityAdaptorGaitCallback(
  const MotionResultSrv::Request::SharedPtr request, MotionResultSrv::Response::SharedPtr response)
{
  INFO("Receive ResultCmd with motion_id: %d", request->motion_id);
  gait_motion_id = request->motion_id;
  gait_shape_value = request->value;
  gait_step_height = request->step_height;
  cmd_source = request->cmd_source;

  response->result = true;
  response->motion_id = request->motion_id;
}

void VelocityAdaptor::PublishCommandVelocity(geometry_msgs::msg::Twist::SharedPtr msg)
{
  std::vector<float> vel_des;
  if (fabs(msg->linear.x) < 5e-3 &&
    fabs(msg->linear.y) < 5e-3 &&
    fabs(msg->angular.z) < 5e-3)
  {
    if(!stop_vel_occur_){
      stop_vel_occur_ = true;
      vel_des = std::vector<float> {
        static_cast<float>(0.0),
        static_cast<float>(0.0),
        static_cast<float>(0.0)
      };
    }else{
      return;
    }
  }else{
    stop_vel_occur_ = false;
    geometry_msgs::msg::TwistStamped twist_stamp;
    auto current_time = now();
    twist_stamp.header.stamp = current_time;
    if (!twist_history_.empty()) {
      auto front_time = rclcpp::Time(twist_history_.front().header.stamp);

      while (current_time - front_time > twist_history_duration_) {
        const auto & front_twist = twist_history_.front();
        twist_cumulate_.linear.x -= front_twist.twist.linear.x;
        twist_cumulate_.linear.y -= front_twist.twist.linear.y;
        twist_cumulate_.angular.z -= front_twist.twist.angular.z;
        twist_history_.pop_front();

        if (twist_history_.empty()) {
          break;
        }

        front_time = rclcpp::Time(twist_history_.front().header.stamp);
      }
    }
    twist_stamp.twist.linear.x = msg->linear.x;
    twist_stamp.twist.linear.y = msg->linear.y;
    twist_stamp.twist.angular.z = msg->angular.z;

    twist_history_.push_back(twist_stamp);
    
    const auto & back_twist = twist_history_.back();
    twist_cumulate_.linear.x += back_twist.twist.linear.x;
    twist_cumulate_.linear.y += back_twist.twist.linear.y;
    twist_cumulate_.angular.z += back_twist.twist.angular.z; 

    vel_des = std::vector<float> {
      static_cast<float>(twist_cumulate_.linear.x / twist_history_.size()),
      static_cast<float>(twist_cumulate_.linear.y / twist_history_.size()),
      static_cast<float>(twist_cumulate_.angular.z / twist_history_.size())
    }; 
  
  }

  ::protocol::msg::MotionServoCmd command;
  command.cmd_source = 4;
  command.motion_id = gait_motion_id;
  command.vel_des = vel_des;
  command.value = gait_shape_value;
  command.step_height = gait_step_height;
  motion_vel_cmd_pub_->publish(command);
}

}  // namespace navigation
}  // namespace cyberdog
