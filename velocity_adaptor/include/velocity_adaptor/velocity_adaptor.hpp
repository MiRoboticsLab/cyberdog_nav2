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

#ifndef VELOCITY_ADAPTOR__VELOCITY_ADAPTOR_HPP_
#define VELOCITY_ADAPTOR__VELOCITY_ADAPTOR_HPP_

#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/float32.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "protocol/msg/motion_servo_cmd.hpp"
#include "protocol/msg/motion_servo_response.hpp"
#include "protocol/srv/motion_result_cmd.hpp"
#include "cyberdog_common/cyberdog_log.hpp"

namespace cyberdog
{

namespace navigation
{

class VelocityAdaptor : public ::rclcpp::Node
{
  using MotionResultSrv = protocol::srv::MotionResultCmd;

public:
  VelocityAdaptor();
  ~VelocityAdaptor();

private:
  /**
   * @brief Receive Nav2 publish cmd_vel topic data
   *
   * @param msg
   */
  void HandleNavCommandVelocity(geometry_msgs::msg::Twist::SharedPtr msg);

  /**
   * @brief
   *
   * @param msg
   */
  void PublishCommandVelocity(geometry_msgs::msg::Twist::SharedPtr msg);

  /**
   * @brief
   *
   * @param request
   * @param response
   */
  void VelocityAdaptorGaitCallback(
    const MotionResultSrv::Request::SharedPtr request,
    MotionResultSrv::Response::SharedPtr response);

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr nav_cmd_vel_sub_ {nullptr};
  rclcpp::Publisher<::protocol::msg::MotionServoCmd>::SharedPtr motion_vel_cmd_pub_ {nullptr};
  rclcpp::Service<MotionResultSrv>::SharedPtr change_gait_srv_ {nullptr};

  int32_t gait_motion_id;
  int32_t gait_shape_value;
  int32_t cmd_source;
  std::vector<float> gait_step_height;
};

}  // namespace navigation
}  // namespace cyberdog

#endif  // VELOCITY_ADAPTOR__VELOCITY_ADAPTOR_HPP_
