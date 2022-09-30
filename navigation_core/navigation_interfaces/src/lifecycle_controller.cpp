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

#include <string>
#include <memory>

#include "motion_core/lifecycle_controller.hpp"
#include "cyberdog_common/cyberdog_log.hpp"

namespace cyberdog
{
namespace controller
{

LifecycleController::LifecycleController(const std::string & node_name)
  : node_name_{node_name}
{
  sensor_controller_ = std::make_shared<nav2_util::LifecycleServiceClient>(node_name);
}

LifecycleController::~LifecycleController()
{
  Cleanup();
}

bool LifecycleController::Startup()
{
  // Checker node activate state
  if (sensor_controller_->get_state() == lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE) {
    INFO("Current node(%s) has activate state", node_name().c_str());
    return true;
  }

  // Set node configure state
  if (!sensor_controller_->change_state(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE) ) {
    INFO("Set current node(%s) configure state failed", node_name().c_str());
    return false;
  }

  // Set node activate state
  if (!sensor_controller_->change_state(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE) ) {
    INFO("Set current node(%s) activate state failed", node_name().c_str());
    return false;
  }

  INFO("Set current node(%s) activate state success", node_name().c_str());
  return true;
}

bool LifecycleController::Pause()
{
  // Checker node deactivate state
  if (sensor_controller_->get_state() == lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE) {
    INFO("Current node has(%s) deactivate state", node_name().c_str());
    return true;
  }

  // Set node deactivate state
  if (!sensor_controller_->change_state(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE) ) {
    INFO("Set current node(%s) deactivate state failed", node_name().c_str());
    return false;
  }
  INFO("Set current node(%s) deactivate state success", node_name().c_str());
  return true;
}

bool LifecycleController::Cleanup()
{
  // Set node cleanup state
  if (!sensor_controller_->change_state(lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP) ) {
    INFO("Set current node(%s) cleanup state failed", node_name().c_str());
    return false;
  }

  INFO("Set current node(%s) cleanup state success", node_name().c_str());
  return true;
}

std::string LifecycleController::node_name()
{
  return node_name_;
}

}  // namespace controller
}  // namespace cyberdog
