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

#ifndef BEHAVIOR_MANAGER__EXECUTOR_AUTO_TRACKING_HPP_
#define BEHAVIOR_MANAGER__EXECUTOR_AUTO_TRACKING_HPP_

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int8.hpp"
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "protocol/srv/motion_result_cmd.hpp"
#include "cyberdog_debug/backtrace.hpp"
namespace cyberdog
{
namespace algorithm
{

class ExecutorAutoTracking
{
public:
  explicit ExecutorAutoTracking(const std::string & node_name)
  {
    node_ = std::make_shared<rclcpp::Node>(node_name);
    std::thread{[this]{rclcpp::spin(node_);}}.detach();
  }
  ~ExecutorAutoTracking(){}
  /**
   * @brief 
   * trigger=true时启动自主遛狗，trigger=false时停止自主遛狗
   * 
   * @param trigger 
   */
  void Execute(bool trigger)
  {
    std::unique_lock<std::mutex> lk(auto_tracking_start_mutex_);
    auto_tracking_start_ = trigger;
    if(auto_tracking_start_) {
      auto_tracking_start_cv_.notify_one();
    }
  }
private:
  void DoAutoTracking()
  {
    while (rclcpp::ok())
    {
      if (!auto_tracking_start_) {
        std::unique_lock<std::mutex> lk(auto_tracking_start_mutex_);
        auto_tracking_start_cv_.wait(lk);
        auto_tracking_start_ = true;
      }
      // 按顺序执行不同的动作列表，围绕目标转圈的动作能被打断
      WalkAround();
      // 其他的动作实现接口
      // Foo0();
      // Foo1();
    }
  }
  bool WalkAround()
  {}
  // 其他的动作实现接口
  // Foo0()
  // Foo1();

  rclcpp::Node::SharedPtr node_;
  std::mutex auto_tracking_start_mutex_;
  std::condition_variable auto_tracking_start_cv_;
  bool auto_tracking_start_{false};
};  // class ExecutorAutoTracking
}  // namespace algorithm
}  // namespace cyberdog
#endif  // BEHAVIOR_MANAGER__EXECUTOR_AUTO_TRACKING_HPP_
