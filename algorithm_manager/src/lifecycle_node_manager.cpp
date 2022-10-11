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

#include "algorithm_manager/lifecycle_node_manager.hpp"
#include "cyberdog_common/cyberdog_log.hpp"

namespace cyberdog
{
namespace algorithm
{

LifecycleNodeManager::LifecycleNodeManager(const std::string & node_name)
: node_name_{node_name}
{
  node_controller_ = std::make_shared<nav2_util::LifecycleServiceClient>(node_name);
}

LifecycleNodeManager::~LifecycleNodeManager()
{
  Cleanup();
}

bool LifecycleNodeManager::Configure()
{
  // Set node configure state
  if (!node_controller_->change_state(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE) ) {
    ERROR("Set current lifecycle node(%s) configure state failed", node_name().c_str());
    return false;
  }

  INFO("Set current lifecycle node(%s) configure state success", node_name().c_str());
  return true;
}

bool LifecycleNodeManager::Startup()
{
  // Checker node activate state
  if (node_controller_->get_state() == lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE) {
    WARN("Current lifecycle node(%s) has activate state", node_name().c_str());
    return true;
  }

  // Set node activate state
  if (!node_controller_->change_state(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE) ) {
    ERROR("Set current lifecycle node(%s) activate state failed", node_name().c_str());
    return false;
  }

  INFO("Set current lifecycle node(%s) activate state success", node_name().c_str());
  return true;
}

bool LifecycleNodeManager::Pause()
{
  // Checker node deactivate state
  if (node_controller_->get_state() == lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE) {
    WARN("Current lifecycle node has(%s) deactivate state", node_name().c_str());
    return true;
  }

  // Set node deactivate state
  if (!node_controller_->change_state(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE) ) {
    ERROR("Set current lifecycle node(%s) deactivate state failed", node_name().c_str());
    return false;
  }

  INFO("Set current lifecycle node(%s) deactivate state success", node_name().c_str());
  return true;
}

bool LifecycleNodeManager::Cleanup()
{
  // Set node cleanup state
  if (!node_controller_->change_state(lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP) ) {
    INFO("Set current lifecycle node(%s) cleanup state failed", node_name().c_str());
    return false;
  }

  INFO("Set current lifecycle node(%s) cleanup state success", node_name().c_str());
  return true;
}

std::string LifecycleNodeManager::node_name()
{
  return node_name_;
}

}  // namespace algorithm
}  // namespace cyberdog
