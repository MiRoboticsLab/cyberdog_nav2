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

#include "algorithm_manager/lifecycle_controller.hpp"
#include "cyberdog_common/cyberdog_log.hpp"

namespace cyberdog
{
namespace algorithm
{

LifecycleController::LifecycleController(const std::string & node_name)
: node_name_{node_name}
{
  node_controller_ = std::make_shared<nav2_util::LifecycleServiceClient>(node_name);
}

LifecycleController::~LifecycleController()
{
}

bool LifecycleController::IsConfigure(const int timeout)
{
  if (!node_controller_->service_exist(std::chrono::seconds(2))) {
    ERROR("Lifecycle [%s] not exist", node_name().c_str());
    return false;
  }

  bool is_timeout = false;
  auto state = node_controller_->get_state(is_timeout, timeout);
  if (is_timeout) {
    ERROR("Cannot get state of [%s]", node_name().c_str());
    return false;
  }

  return state == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE;
}

bool LifecycleController::IsActivate(const int timeout)
{
  if (!node_controller_->service_exist(std::chrono::seconds(2))) {
    ERROR("Lifecycle [%s] not exist", node_name().c_str());
    return false;
  }

  bool is_timeout = false;
  auto state = node_controller_->get_state(is_timeout, timeout);
  if (is_timeout) {
    ERROR("Cannot get state of [%s]", node_name().c_str());
    return false;
  }
  return state == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;
}

bool LifecycleController::IsDeactivate(const int timeout)
{
  if (!node_controller_->service_exist(std::chrono::seconds(2))) {
    ERROR("Lifecycle [%s] not exist", node_name().c_str());
    return false;
  }

  bool is_timeout = false;
  auto state = node_controller_->get_state(is_timeout, timeout);
  if (is_timeout) {
    ERROR("Cannot get state of [%s]", node_name().c_str());
    return false;
  }
  return state == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE;
}

bool LifecycleController::Configure(const int timeout)
{
  // Checker node configure state
  if (IsConfigure(timeout)) {
    WARN("Current lifecycle node(%s) has configure state", node_name().c_str());
    return true;
  }

  // Set node configure state
  if (!node_controller_->change_state(
      lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE,
      timeout) )
  {
    ERROR("Set current lifecycle node(%s) configure state failed", node_name().c_str());
    return false;
  }

  INFO("Set current lifecycle node(%s) configure state success", node_name().c_str());
  return true;
}

bool LifecycleController::Startup(const int timeout)
{
  if (IsActivate(timeout)) {
    WARN("Current lifecycle node(%s) has activate state", node_name().c_str());
    return true;
  }

  // Set node activate state
  if (!node_controller_->change_state(
      lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE,
      timeout) )
  {
    ERROR("Set current lifecycle node(%s) activate state failed", node_name().c_str());
    return false;
  }

  INFO("Set current lifecycle node(%s) activate state success", node_name().c_str());
  return true;
}

bool LifecycleController::Pause(const int timeout)
{
  // Checker node deactivate state
  if (IsDeactivate(timeout)) {
    WARN("Current lifecycle node(%s) has deactivate state", node_name().c_str());
    return true;
  }

  // Set node deactivate state
  if (!node_controller_->change_state(
      lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE,
      timeout) )
  {
    ERROR("Set current lifecycle node(%s) deactivate state failed", node_name().c_str());
    return false;
  }

  INFO("Set current lifecycle node(%s) deactivate state success", node_name().c_str());
  return true;
}

bool LifecycleController::Cleanup(const int timeout)
{
  if (!node_controller_->service_exist(std::chrono::seconds(2))) {
    ERROR("Lifecycle [%s] not exist", node_name().c_str());
    return false;
  }

  // Checker node cleanup state
  bool is_timeout = false;
  auto state = node_controller_->get_state(is_timeout, timeout);
  if (is_timeout) {
    ERROR("Cannot get state of [%s]", node_name().c_str());
    return false;
  }

  if (state == lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED) {
    WARN("Current lifecycle node (%s) has cleanup state", node_name().c_str());
    return true;
  }

  // Set node cleanup state
  if (!node_controller_->change_state(
      lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP,
      timeout) )
  {
    INFO("Set current lifecycle node(%s) cleanup state failed", node_name().c_str());
    return false;
  }

  INFO("Set current lifecycle node(%s) cleanup state success", node_name().c_str());
  return true;
}

std::string LifecycleController::node_name()
{
  return node_name_;
}

}  // namespace algorithm
}  // namespace cyberdog
