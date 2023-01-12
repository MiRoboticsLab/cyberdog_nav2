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

#ifndef POSITIONCHECKER__POSITION_CHECKER_NODE_HPP_
#define POSITIONCHECKER__POSITION_CHECKER_NODE_HPP_

#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <atomic>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_util/robot_utils.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_msgs/msg/bool.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/create_timer_ros.h"
#include "tf2_ros/transform_listener.h"

using SetBool = std_srvs::srv::SetBool;
namespace CYBERDOG_NAV
{
class PositionChecker : public rclcpp::Node
{
public:
  PositionChecker();
  ~PositionChecker();

private:
  void HandleTriggerCallback(const std_msgs::msg::Bool::SharedPtr msg);

  void HandlePoseServerStatus(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response);

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  void loop();
  bool looping_;
  std::shared_ptr<std::thread> loop_thread_;
  rclcpp::Service<SetBool>::SharedPtr enable_service;
  void serviceCallback(
    const std::shared_ptr<rmw_request_id_t>,
    const std::shared_ptr<SetBool::Request> request,
    std::shared_ptr<SetBool::Response> response);
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pos_pub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enable_sub_ {nullptr};

  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr map_server_state_;
  std::atomic_bool activate_;
};
}  // namespace CYBERDOG_NAV

#endif  // POSITIONCHECKER__POSITION_CHECKER_NODE_HPP_
