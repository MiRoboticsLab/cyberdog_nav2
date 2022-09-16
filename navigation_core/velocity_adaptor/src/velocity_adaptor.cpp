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

namespace cyberdog
{
namespace navigation
{

VelocityAdaptor::VelocityAdaptor()
: Node("velocity_adaptor")
{
  motion_vel_cmd_pub_ = this->create_publisher<::protocol::msg::MotionServoCmd>(
    "motion_servo_cmd", rclcpp::SystemDefaultsQoS());

  nav_cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", rclcpp::SystemDefaultsQoS(),
    std::bind(&VelocityAdaptor::HandleNavCommandVelocity, this, std::placeholders::_1));
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

void VelocityAdaptor::PublishCommandVelocity(geometry_msgs::msg::Twist::SharedPtr msg)
{
  // INFO("SetCommandVelocity");
  std::vector<float> vel_des {
    static_cast<float>(msg->linear.x),
    static_cast<float>(msg->linear.y),
    static_cast<float>(msg->angular.z)
  };

  std::vector<float> step_height {
    0.05,
    0.05
  };

  ::protocol::msg::MotionServoCmd command;
  command.motion_id = 303;
  command.vel_des = vel_des;
  command.step_height = step_height;
  motion_vel_cmd_pub_->publish(command);
}


}  // namespace navigation
}  // namespace cyberdog
