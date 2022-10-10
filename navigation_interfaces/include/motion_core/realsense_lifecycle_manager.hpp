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

#ifndef MOTION_CORE__REALSENSE_LIFECYCLE_MANAGER_HPP_
#define MOTION_CORE__REALSENSE_LIFECYCLE_MANAGER_HPP_

#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rcutils/logging_macros.h"
#include "nav2_util/service_client.hpp"

using namespace std::chrono_literals;    // NOLINT

namespace carpo_navigation
{


class RealSenseLifecycleServiceClient : public rclcpp::Node
{
public:
  explicit RealSenseLifecycleServiceClient(const std::string & node_name);
  ~RealSenseLifecycleServiceClient();

  void init();

  /// Requests the current state of the node
  /**
   * In this function, we send a service request
   * asking for the current state of the node
   * lc_talker.
   * If it does return within the given time_out,
   * we return the current state of the node, if
   * not, we return an unknown state.
   * \param time_out Duration in seconds specifying
   * how long we wait for a response before returning
   * unknown state
   */
  unsigned int GetState(std::chrono::seconds time_out = 5s);

  /// Invokes a transition
  /**
   * We send a Service request and indicate
   * that we want to invoke transition with
   * the id "transition".
   * By default, these transitions are
   * - configure
   * - activate
   * - cleanup
   * - shutdown
   * \param transition id specifying which
   * transition to invoke
   * \param time_out Duration in seconds specifying
   * how long we wait for a response before returning
   * unknown state
   */
  bool ChangeState(std::uint8_t transition, std::chrono::seconds time_out = 5s);

  /**
   * @brief configure
   *
   * @return true
   * @return false
   */
  bool Startup();

  /**
   * @brief deactivate
   *
   * @return true
   * @return false
   */
  bool Pause();

  /**
   * @brief cleanup
   *
   * @return true
   * @return false
   */
  bool Cleanup();

private:
  std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::GetState>> client_get_state_;
  std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::ChangeState>> client_change_state_;

  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
};

}  // namespace carpo_navigation
#endif  // MOTION_CORE__REALSENSE_LIFECYCLE_MANAGER_HPP_
