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
#ifndef ALGORITHM_MANAGER__EXECUTOR_RESET_NAV_HPP_
#define ALGORITHM_MANAGER__EXECUTOR_RESET_NAV_HPP_

#include <memory>
#include <string>
#include "cyberdog_common/cyberdog_log.hpp"
#include "algorithm_manager/executor_ab_navigation.hpp"
#include "algorithm_manager/executor_auto_dock.hpp"
#include "algorithm_manager/executor_laser_localization.hpp"
#include "algorithm_manager/executor_laser_mapping.hpp"
#include "algorithm_manager/executor_uwb_tracking.hpp"
#include "algorithm_manager/executor_vision_tracking.hpp"
#include "algorithm_manager/executor_base.hpp"

namespace cyberdog
{
namespace algorithm
{

class ExecutorResetNav : public ExecutorBase
{
public:
  explicit ExecutorResetNav(std::string node_name)
  : ExecutorBase(node_name)
  {
    nav_stop_trigger_pub_ = create_publisher<std_msgs::msg::Bool>("stop_nav_trigger", 10);

    // spin
    std::thread{[this]() {
        rclcpp::spin(this->get_node_base_interface());
      }
    }.detach();
  }

  void Start(AlgorithmMGR::Goal::ConstSharedPtr goal) override
  {
    (void)goal;
    ERROR("Error: Start ExecutorResetNav should never be called");
  }

  void Stop(
    const StopTaskSrv::Request::SharedPtr request,
    StopTaskSrv::Response::SharedPtr response) override
  {
    (void)request;
    DeactivateDepsLifecycleNodes(this->get_name());
    response->result = OperateDepsNav2LifecycleNodes(this->get_name(), Nav2LifecycleMode::kPause) ?
      StopTaskSrv::Response::SUCCESS :
      StopTaskSrv::Response::FAILED;
    INFO("Nav Reset");

    PublishResetNavTrigger();
  }
  void Cancel() override
  {
    ERROR("Error: Cancel ExecutorResetNav should never be called");
  }
  // void UpdateStatus(const ExecutorStatus & executor_status) override;
  // void GetFeedback(protocol::action::Navigation::Feedback::SharedPtr feedback) override;

private:
  /**
   * @brief Reset all nav default state
   */
  void PublishResetNavTrigger()
  {
    std_msgs::msg::Bool data;
    data.data = true;
    nav_stop_trigger_pub_->publish(data);
  }

  // vision mapping alive
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr nav_stop_trigger_pub_{nullptr};
};  // class ExecutorLaserMapping
}  // namespace algorithm
}  // namespace cyberdog
#endif  // ALGORITHM_MANAGER__EXECUTOR_RESET_NAV_HPP_
