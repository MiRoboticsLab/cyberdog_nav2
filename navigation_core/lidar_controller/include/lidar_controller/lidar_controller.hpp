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


#ifndef LIDAR_CONTROLLER__LIDAR_CONTROLLER_HPP_
#define LIDAR_CONTROLLER__LIDAR_CONTROLLER_HPP_

#include <functional>
#include <memory>
#include <string>

#include "nav2_util/lifecycle_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"
using TRIGGERT = std_srvs::srv::SetBool;
namespace CYBERDOG_NAV
{
class LidarController : public nav2_util::LifecycleNode
{
public:
  LidarController();
  ~LidarController();

protected:
  /**
   * @brief Sets up required params and services. Loads map and its parameters
   * from the file
   * @param state Lifecycle Node's state
   * @return Success or Failure
   */
  nav2_util::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Enable lidar by publish topic
   * @param state Lifecycle Node's state
   * @return Success or Failure
   */
  nav2_util::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Disable lidar by publish topic
   * @param state Lifecycle Node's state
   * @return Success or Failure
   */
  nav2_util::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Resets the member variables
   * @param state Lifecycle Node's state
   * @return Success or Failure
   */
  nav2_util::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Called when in Shutdown state
   * @param state Lifecycle Node's state
   * @return Success or Failure
   */
  nav2_util::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State & state) override;

private:
  // A topic on which the control cmd will be published
  rclcpp::Client<TRIGGERT>::SharedPtr switch_cli_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
};
}  // namespace CYBERDOG_NAV
#endif  // LIDAR_CONTROLLER__LIDAR_CONTROLLER_HPP_
