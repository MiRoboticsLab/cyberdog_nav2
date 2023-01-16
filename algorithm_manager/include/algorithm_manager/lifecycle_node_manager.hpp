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

#ifndef ALGORITHM_MANAGER__LIFECYCLE_NODE_MANAGER_HPP_
#define ALGORITHM_MANAGER__LIFECYCLE_NODE_MANAGER_HPP_

#include <string>
#include <memory>
#include <mutex>

#include "nav2_util/lifecycle_service_client.hpp"
#include "nav2_msgs/srv/manage_lifecycle_nodes.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "algorithm_manager/lifecycle_controller.hpp"

using namespace std::chrono_literals;    // NOLINT

namespace cyberdog
{
namespace algorithm
{
// class LifecycleNodeManager
// {
// public:
//   explicit LifecycleNodeManager(const std::string & node_name);
//   ~LifecycleNodeManager();

//   /**
//    * @brief Lifecycle set configure state
//    *
//    * @return true
//    * @return false
//    */
//   bool Configure();

//   /**
//    * @brief Lifecycle set configure state
//    *
//    * @return true
//    * @return false
//    */
//   bool Startup();

//   /**
//    * @brief Lifecycle set deactivate state
//    *
//    * @return true
//    * @return false
//    */
//   bool Pause();

//   /**
//    * @brief Lifecycle set cleanup state
//    *
//    * @return true
//    * @return false
//    */
//   bool Cleanup();

// private:
//   // Get current lifecycle node name
//   std::string node_name();

//   // change state
//   std::shared_ptr<nav2_util::LifecycleServiceClient> node_controller_ {nullptr};

//   // Record node's name
//   std::string node_name_;
// };


class LifecycleNodeManager
{
public:
  /**
   * @brief Which lifecycle node set state
   */
  enum class LifeCycleNode
  {
    RealSenseCameraSensor,
    RGBCameraSensor,
  };

  /**
   * @brief Get the Singleton object
   *
   * @return std::shared_ptr<LifecycleNodeManager>
   */
  static std::shared_ptr<LifecycleNodeManager> GetSingleton();

  /**
   * @brief LifeCycle configure state
   *
   * @param which which lifecycle node set configure state
   * @return true Success
   * @return false Failure
   */
  static bool Configure(const LifeCycleNode & which, const std::chrono::seconds timeout = 3s);

  /**
   * @brief LifeCycle activate state
   *
   * @param which which lifecycle node set activate state
   * @return true Success
   * @return false Failure
   */
  static bool Startup(const LifeCycleNode & which, const std::chrono::seconds timeout = 3s);

  /**
   * @brief LifeCycle deactivate state
   *
   * @param which which lifecycle node set deactivate state
   * @return true Success
   * @return false Failure
   */
  static bool Pause(const LifeCycleNode & which, const std::chrono::seconds timeout = 3s);

  /**
   * @brief LifeCycle cleanup state
   *
   * @param which which lifecycle node set cleanup state
   * @return true Success
   * @return false Failure
   */
  static bool Cleanup(const LifeCycleNode & which, const std::chrono::seconds timeout = 3s);

  /**
   * @brief Check LifeCycle state
   *
   * @param which Get which lifecycle node state
   * @return true Success
   * @return false Failure
   */
  static bool IsActivate(const LifeCycleNode & which, const std::chrono::seconds timeout = 3s);

private:
  LifecycleNodeManager();

  // RealSense Lifecycle
  static std::shared_ptr<LifecycleController> realsense_lifecycle_;

  // RGB-G camera Lifecycle
  static std::shared_ptr<LifecycleController> camera_lifecycle_;
};


}  // namespace algorithm
}  // namespace cyberdog
#endif  // ALGORITHM_MANAGER__LIFECYCLE_NODE_MANAGER_HPP_
