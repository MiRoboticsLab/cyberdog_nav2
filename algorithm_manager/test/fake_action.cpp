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

#include <memory>
#include <vector>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
// #include "algorithm_manager/algorithm_task_manager.hpp"
#include "cyberdog_common/cyberdog_log.hpp"
#include "mcr_msgs/action/target_tracking.hpp"
#include "cyberdog_debug/backtrace.hpp"
namespace cyberdog
{
namespace algorithm
{

class FakeActionServer : public rclcpp::Node
{
public:
  FakeActionServer() : rclcpp::Node("fake_action")
  {
    navigation_server_ = rclcpp_action::create_server<mcr_msgs::action::TargetTracking>(
      this, "tracking_target_fake",
      std::bind(
        &FakeActionServer::HandleAlgorithmManagerGoal,
        this, std::placeholders::_1, std::placeholders::_2),
      std::bind(
        &FakeActionServer::HandleAlgorithmManagerCancel,
        this, std::placeholders::_1),
      std::bind(
        &FakeActionServer::HandleAlgorithmManagerAccepted,
        this, std::placeholders::_1));
    // std::thread{std::bind(&FakeActionServer::GetExecutorStatus, this)}.detach();
  }
  ~FakeActionServer(){}

private:
  rclcpp_action::GoalResponse HandleAlgorithmManagerGoal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const mcr_msgs::action::TargetTracking::Goal> goal)
    {
      (void)uuid;
      (void)goal;
      INFO("Accept goal");
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }
  rclcpp_action::CancelResponse HandleAlgorithmManagerCancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<mcr_msgs::action::TargetTracking>> goal_handle)
    {
      INFO("Received request to cancel goal");
      (void)goal_handle;
      return rclcpp_action::CancelResponse::ACCEPT;
    }
  void HandleAlgorithmManagerAccepted(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<mcr_msgs::action::TargetTracking>> goal_handle)
    {
      goal_handle_ = goal_handle;
      std::thread{std::bind(&FakeActionServer::TaskExecute, this)}.detach();
    }
  void TaskExecute()
  { 
    auto feedback = std::make_shared<mcr_msgs::action::TargetTracking::Feedback>();
    auto result = std::make_shared<mcr_msgs::action::TargetTracking::Result>();
    while (rclcpp::ok()) {
      if(goal_handle_->is_canceling()) {
        goal_handle_->canceled(result);
        return;
      }
      INFO("Running");
      if (feedback->exception_code > 5) {
        feedback->exception_code = 1;
      } else {
        feedback->exception_code++;
      }
      goal_handle_->publish_feedback(feedback);
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
  }
  rclcpp_action::Server<mcr_msgs::action::TargetTracking>::SharedPtr navigation_server_;
  std::shared_ptr<rclcpp_action::ServerGoalHandle<mcr_msgs::action::TargetTracking>> goal_handle_;
  // std::shared_ptr<ExecutorBase> activated_executor_;
  // std::shared_ptr<ExecutorAbNavigation> executor_ab_navigation_;
  // std::shared_ptr<ExecutorAutoDock> executor_auto_dock_;
  // std::shared_ptr<ExecutorLaserMapping> executor_laser_mapping_;
  // std::shared_ptr<ExecutorLaserLocalization> executor_laser_localization_;
  // std::shared_ptr<ExecutorUwbTracking> executor_uwb_tracking_;
  // std::shared_ptr<ExecutorVisionTracking> executor_vision_tracking_;
  // std::condition_variable executor_start_cv_, executor_status_cv_;
  // std::mutex executor_start_mutex_, executor_status_mutex_;
};  // class algorithm_manager
}  // namespace algorithm
}  // namespace cyberdog

int main(int argc, char ** argv)
{
  LOGGER_MAIN_INSTANCE("AlgorithmTaskManager");
  cyberdog::debug::register_signal();
  rclcpp::init(argc, argv);
  auto atm = std::make_shared<cyberdog::algorithm::FakeActionServer>();
  rclcpp::spin(atm);
}
