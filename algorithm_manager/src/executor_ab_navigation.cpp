// Copyright (c) 2023 Beijing Xiaomi Mobile Software Co., Ltd. All rights reserved.
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
#include <utility>

#include "algorithm_manager/executor_ab_navigation.hpp"
#include "cyberdog_common/cyberdog_json.hpp"
#include "filesystem/filesystem.hpp"

namespace cyberdog
{
namespace algorithm
{

/**
成功
- 导航启动成功，设置目标点成功，正在规划路径： 300
- 正在导航中： 307
- 到达目标点：308

- 失败
- 地图不存在：301
- 底层导航失败：
- 底层导航功能服务连接失败，请重新发送目标：302
- 发送目标点失败，请重新发送目标：303
- 底层导航功能失败，请重新发送目标：304
- 目标点为空，请重新选择目标：305
- 规划路径失败，请重新选择目标： 306

- 地图检查服务feedback_code ：
  - 正在检查地图：309
  - 地图检查成功： 310
  - 地图不存在，请重新建图： 311

*/

constexpr int kSuccessStartNavigation = 300;      // 导航启动成功，设置目标点成功，正在规划路径： 300  // NOLINT
constexpr int kSuccessStartingNavigation = 307;   // 正在导航中： 307
constexpr int kSuccessArriveTargetGoal = 308;     // 到达目标点：308

constexpr int kErrorConnectActionServer = 302;  // 底层导航功能服务连接失败，请重新发送目标：302
constexpr int kErrorSendGoalTarget = 303;       // 发送目标点失败，请重新发送目标：303
constexpr int kErrorNavigationAbort = 304;      // 底层导航功能失败，请重新发送目标：304
constexpr int kErrorTargetGoalIsEmpty = 305;    // 目标点为空，请重新选择目标：305

constexpr int kMapChecking = 309;
constexpr int kMapCheckingSuccess = 310;
constexpr int kMapErrorNotExist = 311;

ExecutorAbNavigation::ExecutorAbNavigation(std::string node_name)
: ExecutorBase(node_name)
{
  executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  executor_->add_node(this->get_node_base_interface());

  action_client_ =
    rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
    this, "navigate_to_pose");

  // Initialize pubs & subs
  plan_publisher_ = create_publisher<nav_msgs::msg::Path>("plan", 1);

  // Reset navigation service
  callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  stop_running_server_ = this->create_service<std_srvs::srv::SetBool>(
    "stop_running_robot_navigation", std::bind(
      &ExecutorAbNavigation::HandleStopRobotNavCallback, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3),
    rmw_qos_profile_default, callback_group_);

  // spin
  std::thread{[this] {this->executor_->spin();}}.detach();
}

ExecutorAbNavigation::~ExecutorAbNavigation()
{
  nav_client_->reset();
}

void ExecutorAbNavigation::Start(const AlgorithmMGR::Goal::ConstSharedPtr goal)
{
  INFO("AB navigation started");
  Timer timer, total_timer;
  timer.Start();
  total_timer.Start();

  is_exit_ = false;

  // Check current map exits
  UpdateFeedback(kMapChecking);
  bool exist = CheckMapAvailable();
  if (!exist) {
    ERROR("AB navigation can't start up, because current robot's map not exist");
    UpdateFeedback(kMapErrorNotExist);
    task_abort_callback_();
    return;
  }
  UpdateFeedback(kMapCheckingSuccess);
  INFO("[0] Checking map available Elapsed time: %.5f [seconds]", timer.ElapsedSeconds());
  timer.Start();
  // Check all depends is ok
  // 1 正在激活依赖节点
  UpdateFeedback(AlgorithmMGR::Feedback::TASK_PREPARATION_EXECUTING);
  bool ready = IsDependsReady();
  if (!ready) {
    ERROR("AB navigation lifecycle depend start up failed.");
    // 2 激活依赖节点失败
    UpdateFeedback(AlgorithmMGR::Feedback::TASK_PREPARATION_FAILED);
    task_abort_callback_();
    return;
  }
  // 3 激活依赖节点成功
  UpdateFeedback(AlgorithmMGR::Feedback::TASK_PREPARATION_SUCCESS);
  INFO("[1] Activate lifecycle nodes Elapsed time: %.5f [seconds]", timer.ElapsedSeconds());
  timer.Start();
  // Realtime response user stop operation
  if (CheckExit()) {
    WARN("Navigation AB is stop, not need connect action server.");
    return;
  }
  // Check action client connect server
  bool connect = IsConnectServer();
  if (!connect) {
    ERROR("Connect navigation AB point server failed.");
    UpdateFeedback(kErrorConnectActionServer);
    DeactivateDepsLifecycleNodes(20000, true);
    task_abort_callback_();
    return;
  }
  INFO("[2] Connect nav2 action server Elapsed time: %.5f [seconds]", timer.ElapsedSeconds());
  timer.Start();
  // Check input target goal is legal
  bool legal = IsLegal(goal);
  if (!legal) {
    ERROR("Current navigation AB point is not legal.");
    UpdateFeedback(kErrorTargetGoalIsEmpty);
    DeactivateDepsLifecycleNodes(20000, true);
    task_abort_callback_();
    return;
  }
  INFO("[3] Check goal Elapsed time: %.5f [seconds]", timer.ElapsedSeconds());
  timer.Start();
  if (CheckExit()) {
    WARN("Navigation AB is stop, not need start velocity smoother and send target goal.");
    return;
  }

  timer.Start();
  // Print set target goal pose
  Debug2String(goal->poses[0]);

  // Send goal request
  if (!SendGoal(goal->poses[0])) {
    ERROR("Send navigation AB point send target goal request failed.");
    DeactivateDepsLifecycleNodes(20000, true);
    UpdateFeedback(kErrorSendGoalTarget);
    task_abort_callback_();
    return;
  }
  INFO("[5] Sending goal Elapsed time: %.5f [seconds]", timer.ElapsedSeconds());
  // 结束激活进度的上报
  UpdateFeedback(kSuccessStartNavigation);
  INFO("Navigation AB point send target goal request success.");
  INFO("[Total] Start AB navition Elapsed time: %.5f [seconds]", total_timer.ElapsedSeconds());
}

void ExecutorAbNavigation::Stop(
  const StopTaskSrv::Request::SharedPtr request,
  StopTaskSrv::Response::SharedPtr response)
{
  (void)request;
  Timer timer;
  timer.Start();
  response->result = StopTaskSrv::Response::SUCCESS;

  INFO("Navigation AB will stop");
  bool cancel = ShouldCancelGoal();
  if (!cancel) {
    WARN("Current robot can't stop, due to navigation status is not available.");
    nav_goal_handle_.reset();
    return;
  }

  if (nav_goal_handle_ == nullptr) {
    response->result = StopTaskSrv::Response::FAILED;
    return;
  }

  bool success = CancelGoal();
  if (!success) {
    ERROR("Navigation AB will stop timeout.");
  }
  nav_goal_handle_.reset();

  response->result = success ? StopTaskSrv::Response::SUCCESS : StopTaskSrv::Response::FAILED;
  INFO("Navigation AB Stoped success");
  INFO("[Total] Stop AB nav Elapsed time: %.5f [seconds]", timer.ElapsedSeconds());
}

void ExecutorAbNavigation::Cancel()
{
  // Reset lifecycle nodes
  INFO("Navigation AB Canceled");
}

void ExecutorAbNavigation::HandleGoalResponseCallback(
  NavigationGoalHandle::SharedPtr goal_handle)
{
  (void)goal_handle;
  INFO("Navigation AB Goal accepted");
}

void ExecutorAbNavigation::HandleFeedbackCallback(
  NavigationGoalHandle::SharedPtr,
  const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> feedback)
{
  (void)feedback;
  UpdateFeedback(kSuccessStartingNavigation);
}

void ExecutorAbNavigation::HandleResultCallback(
  const NavigationGoalHandle::WrappedResult result)
{
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      INFO("Navigation AB point have arrived target goal success");
      UpdateFeedback(kSuccessArriveTargetGoal);
      task_success_callback_();
      break;
    case rclcpp_action::ResultCode::ABORTED:
      ERROR("Navigation AB run target goal aborted");
      UpdateFeedback(kErrorNavigationAbort);
      ResetPreprocessingValue();
      task_abort_callback_();
      break;
    case rclcpp_action::ResultCode::CANCELED:
      ERROR("Navigation AB run target goal canceled");
      cancel_goal_cv_.notify_one();
      task_cancle_callback_();
      break;
    default:
      ERROR("Navigation AB run target goal unknown result code");
      break;
  }

  PublishZeroPath();
}

bool ExecutorAbNavigation::IsDependsReady()
{
  INFO("IsDependsReady(): Trying to get lifecycle_mutex_");
  std::lock_guard<std::mutex> lock(lifecycle_mutex_);
  INFO("IsDependsReady(): Success to get lifecycle_mutex_");
  // Nav lifecycle
  if (!ActivateDepsLifecycleNodes(this->get_name())) {
    DeactivateDepsLifecycleNodes(20000, true);
    return false;
  }

  // start_lifecycle_depend_finished_ = true;
  INFO("Call function IsDependsReady() finished success.");
  return true;
}

bool ExecutorAbNavigation::IsConnectServer()
{
  if (connect_server_finished_) {
    INFO("Current server have connected.");
    return true;
  }

  std::chrono::seconds timeout(5);
  auto is_action_server_ready = action_client_->wait_for_action_server(timeout);

  if (!is_action_server_ready) {
    ERROR("Navigation action server is not available.");
    return false;
  }

  connect_server_finished_ = true;
  return true;
}

bool ExecutorAbNavigation::IsLegal(const AlgorithmMGR::Goal::ConstSharedPtr goal)
{
  return goal->poses.empty() ? false : true;
}

bool ExecutorAbNavigation::SendGoal(const geometry_msgs::msg::PoseStamped & pose)
{
  // Set orientation
  NormalizedGoal(pose);

  // Send the goal pose
  auto send_goal_options =
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();

  send_goal_options.goal_response_callback =
    std::bind(
    &ExecutorAbNavigation::HandleGoalResponseCallback,
    this, std::placeholders::_1);
  send_goal_options.feedback_callback =
    std::bind(
    &ExecutorAbNavigation::HandleFeedbackCallback,
    this, std::placeholders::_1, std::placeholders::_2);
  send_goal_options.result_callback =
    std::bind(
    &ExecutorAbNavigation::HandleResultCallback,
    this, std::placeholders::_1);


  auto future_goal_handle = action_client_->async_send_goal(
    target_goal_, send_goal_options);
  time_goal_sent_ = this->now();

  std::chrono::seconds timeout{5};
  if (future_goal_handle.wait_for(timeout) != std::future_status::ready) {
    ERROR("Wait navigation server timeout.");
    return false;
  }

  nav_goal_handle_ = future_goal_handle.get();

  if (!nav_goal_handle_) {
    ERROR("Navigation AB Goal was rejected by server");
    return false;
  }
  return true;
}

bool ExecutorAbNavigation::ShouldCancelGoal()
{
  // No need to cancel the goal if goal handle is invalid
  if (!nav_goal_handle_) {
    WARN("No need to cancel the goal because goal handle is null");
    return false;
  }

  // Check if the goal is still executing
  auto status = nav_goal_handle_->get_status();
  NavigationStatus2String(status);

  return status == action_msgs::msg::GoalStatus::STATUS_ACCEPTED ||
         status == action_msgs::msg::GoalStatus::STATUS_EXECUTING;
}

void ExecutorAbNavigation::NormalizedGoal(const geometry_msgs::msg::PoseStamped & pose)
{
  // Normalize the goal pose
  target_goal_.pose = pose;
  target_goal_.pose.header.frame_id = "map";
  target_goal_.pose.pose.orientation.w = 1;
}

void ExecutorAbNavigation::Debug2String(const geometry_msgs::msg::PoseStamped & pose)
{
  INFO("Nav target goal: [%f, %f]", pose.pose.position.x, pose.pose.position.y);
}

void ExecutorAbNavigation::NavigationStatus2String(int8_t status)
{
  if (status == action_msgs::msg::GoalStatus::STATUS_UNKNOWN) {
    INFO("Bt navigation return status : STATUS_UNKNOWN");
  } else if (status == action_msgs::msg::GoalStatus::STATUS_ACCEPTED) {
    INFO("Bt navigation return status : STATUS_ACCEPTED");
  } else if (status == action_msgs::msg::GoalStatus::STATUS_EXECUTING) {
    INFO("Bt navigation return status : STATUS_EXECUTING");
  } else if (status == action_msgs::msg::GoalStatus::STATUS_CANCELING) {
    INFO("Bt navigation return status : STATUS_CANCELING");
  } else if (status == action_msgs::msg::GoalStatus::STATUS_SUCCEEDED) {
    INFO("Bt navigation return status : STATUS_SUCCEEDED");
  } else if (status == action_msgs::msg::GoalStatus::STATUS_CANCELED) {
    INFO("Bt navigation return status : STATUS_CANCELED");
  } else if (status == action_msgs::msg::GoalStatus::STATUS_ABORTED) {
    INFO("Bt navigation return status : STATUS_ABORTED");
  }
}

bool ExecutorAbNavigation::CheckMapAvailable(const std::string &)
{
  // std::string map_filename = "/home/mi/mapping/" + map_name;
  // if (!filesystem::exists(map_filename)) {
  //   ERROR("Navigation's map file is not exist.");
  //   return false;
  // }

  return true;
}

void ExecutorAbNavigation::ResetPreprocessingValue()
{
  connect_server_finished_ = false;
  start_lifecycle_depend_finished_ = false;
  start_velocity_smoother_finished_ = false;
}

void ExecutorAbNavigation::PublishZeroPath()
{
  auto msg = std::make_unique<nav_msgs::msg::Path>();
  msg->poses.clear();
  plan_publisher_->publish(std::move(msg));
}

bool ExecutorAbNavigation::ResetAllLifecyceNodes()
{
  INFO("ResetAllLifecyceNodes(): Trying to get lifecycle_mutex_");
  std::lock_guard<std::mutex> lock(lifecycle_mutex_);
  INFO("ResetAllLifecyceNodes(): Success to get lifecycle_mutex_");
  bool success = DeactivateDepsLifecycleNodes(20000, true);
  return success;
}

bool ExecutorAbNavigation::StopRunningRobot()
{
  if (nav_goal_handle_ == nullptr) {
    ERROR("nav_goal_handle is null when trying to stop NavAB");
    return false;
  }

  return CancelGoal();
}

void ExecutorAbNavigation::HandleStopRobotNavCallback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
  std::shared_ptr<std_srvs::srv::SetBool::Response> respose)
{
  INFO("Handle stop current robot nav running.");
  (void)request_header;
  Timer timer, total_timer;
  timer.Start();
  total_timer.Start();
  respose->success = true;
  bool success = false;

  // 1 Check current in navigation state
  if (!request->data) {
    return;
  }

  is_exit_ = true;

  // check robot shoud can call cancel and stop.
  bool can_cancel = ShouldCancelGoal();
  if (can_cancel) {
    // 2 stop current robot navgation
    INFO("Trying to stop running robot.");
    success = StopRunningRobot();
    if (!success) {
      ERROR("Stop robot exception.");
      respose->success = false;
    } else {
      INFO("Stop running robot success");
    }
  }
  std::this_thread::sleep_for(std::chrono::seconds(2));
  INFO("[0] Canceling nav goal Elapsed time: %.5f [seconds]", timer.ElapsedSeconds());
  timer.Start();
  // 3 deactivate all navigation lifecycle nodes
  success = ResetAllLifecyceNodes();
  if (!success) {
    ERROR("Reset all lifecyce nodes failed.");
    respose->success = false;
  }
  INFO("[1] Deactivate lifecycle nodes Elapsed time: %.5f [seconds]", timer.ElapsedSeconds());
  INFO(
    "[Total] Stop AB nav when resetting Elapsed time: %.5f [seconds]",
    total_timer.ElapsedSeconds());
}

bool ExecutorAbNavigation::CheckExit()
{
  return is_exit_;
}

bool ExecutorAbNavigation::CancelGoal()
{
  auto server_ready = action_client_->wait_for_action_server(std::chrono::seconds(5));
  if (!server_ready) {
    ERROR("Navigation action server(navigate_to_pose) is not available.");
    return false;
  }

  try {
    // 判断future_cancel的结果和等待
    std::unique_lock<std::mutex> lock(cancel_goal_mutex_);
    INFO("CancelGoal: run async_cancel_goal request");
    auto future_cancel = action_client_->async_cancel_goal(nav_goal_handle_);
    if (cancel_goal_cv_.wait_for(lock, 10s) == std::cv_status::timeout) {
      INFO("Get cancel_goal_cv_ value: false");
      cancel_goal_result_ = false;
    } else {
      cancel_goal_result_ = true;
      INFO("Get cancel_goal_cv_ value: true");
    }
  } catch (const std::exception & e) {
    ERROR("%s", e.what());
  }

  // clear robot path
  PublishZeroPath();
  return cancel_goal_result_;
}

}  // namespace algorithm
}  // namespace cyberdog
