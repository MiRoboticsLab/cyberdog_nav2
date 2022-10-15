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

#ifndef ALGORITHM_MANAGER__LIFECYCLE_CONTROLLER_HPP_
#define ALGORITHM_MANAGER__LIFECYCLE_CONTROLLER_HPP_

#include <string>
#include <memory>

#include "nav2_util/lifecycle_service_client.hpp"
#include "nav2_msgs/srv/manage_lifecycle_nodes.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"

namespace cyberdog
{
namespace algorithm
{
class LifecycleController
{
public:
  explicit LifecycleController(const std::string & node_name);
  ~LifecycleController();

  /**
   * @brief Check lifecycle is configure state
   * 
   * @return true Success
   * @return false Failure
   */
  bool IsConfigure();

  /**
   * @brief Check lifecycle is activate state
   * 
   * @return true Success
   * @return false Failure
   */
  bool IsActivate();

  /**
   * @brief Check lifecycle is deactivate state
   * 
   * @return true Success
   * @return false Failure
   */
  bool IsDeactivate();

  /**
   * @brief Lifecycle set configure state
   *
   * @return true
   * @return false
   */
  bool Configure();

  /**
   * @brief Lifecycle set configure state
   *
   * @return true
   * @return false
   */
  bool Startup();

  /**
   * @brief Lifecycle set deactivate state
   *
   * @return true
   * @return false
   */
  bool Pause();

  /**
   * @brief Lifecycle set cleanup state
   *
   * @return true
   * @return false
   */
  bool Cleanup();

private:
  // Get current lifecycle node name
  std::string node_name();

  // change state
  std::shared_ptr<nav2_util::LifecycleServiceClient> node_controller_ {nullptr};

  // Record node's name
  std::string node_name_;
};

}  // namespace algorithm
}  // namespace cyberdog
#endif  // ALGORITHM_MANAGER__LIFECYCLE_CONTROLLER_HPP_
