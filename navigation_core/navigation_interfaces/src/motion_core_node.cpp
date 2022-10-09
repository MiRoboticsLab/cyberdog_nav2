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

#include "motion_core/motion_core_node.hpp"
#include "cyberdog_common/cyberdog_log.hpp"

namespace carpo_navigation
{
using namespace std::chrono_literals;
NavigationCore::NavigationCore()
: rclcpp::Node("NavigationCore"),
  server_timeout_(2000),
  client_nav_("lifecycle_manager_navigation"),
  client_loc_("lifecycle_manager_localization"),
  client_mapping_("lifecycle_manager_mapping"),
  client_mcr_uwb_("lifecycle_manager_mcr_uwb"),
  status_(GoalStatus::STATUS_UNKNOWN),
  action_type_(kActionNone),
  task_state_(TaskState::Unknown)
{
  auto options = rclcpp::NodeOptions().arguments(
    {"--ros-args --remap __node:=navigation_dialog_action_client"});
  client_node_ = std::make_shared<rclcpp::Node>("_", options);

  navigation_action_client_ =
    rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
    client_node_, "navigate_to_pose");
  waypoint_follower_action_client_ =
    rclcpp_action::create_client<nav2_msgs::action::FollowWaypoints>(
    client_node_, "follow_waypoints");
  nav_through_poses_action_client_ =
    rclcpp_action::create_client<nav2_msgs::action::NavigateThroughPoses>(
    client_node_, "navigate_through_poses");
  target_tracking_action_client_ =
    rclcpp_action::create_client<mcr_msgs::action::TargetTracking>(
    client_node_, "tracking_target");
  // TODO(PDF):
  client_realsense_manager_ =
    std::make_shared<nav2_util::LifecycleServiceClient>("camera/camera");
  client_vision_manager_ =
    std::make_shared<nav2_util::LifecycleServiceClient>("vision_manager");
  client_tracking_manager_ =
    std::make_shared<nav2_util::LifecycleServiceClient>("tracking");
  client_vision_algo_ =
    client_node_->create_client<protocol::srv::AlgoManager>("algo_manager");
  // Create service server
  service_tracking_object_ = create_service<BodyRegionT>(
    "tracking_object_srv", std::bind(
      &NavigationCore::TrackingSrv_callback, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  client_tracking_object_ = create_client<BodyRegionT>("tracking_object");

  navigation_goal_ = nav2_msgs::action::NavigateToPose::Goal();
  waypoint_follower_goal_ = nav2_msgs::action::FollowWaypoints::Goal();
  nav_through_poses_goal_ = nav2_msgs::action::NavigateThroughPoses::Goal();
  target_tracking_goal_ = mcr_msgs::action::TargetTracking::Goal();

  realsense_lifecycle_controller_ = std::make_unique<LifecycleController>("camera/camera");
  vision_mapping_lifecycle_controller_ = std::make_unique<LifecycleController>("mivinsmapping");
  vision_localization_lifecycle_controller_ =
    std::make_unique<LifecycleController>("mivinslocalization");

  navigation_server_ = rclcpp_action::create_server<Navigation>(
    this, "CyberdogNavigation",
    std::bind(&NavigationCore::HandleNavigationGoal, this, _1, _2),
    std::bind(&NavigationCore::HandleNavigationCancel, this, _1),
    std::bind(&NavigationCore::HandleNavigationAccepted, this, _1));

  callback_group_ =
    this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  start_mapping_client_ = create_client<TriggerT>(
    "start_mapping", rmw_qos_profile_services_default, callback_group_);
  stop_mapping_client_ = create_client<visualization::srv::Stop>(
    "stop_mapping", rmw_qos_profile_services_default, callback_group_);
  // stop_mapping_client_ = create_client<TriggerT>(
  //   "stop_mapping", rmw_qos_profile_services_default, callback_group_);
  start_loc_client_ = create_client<TriggerT>(
    "start_location", rmw_qos_profile_services_default, callback_group_);
  stop_loc_client_ = create_client<TriggerT>(
    "stop_location", rmw_qos_profile_services_default, callback_group_);
  realtime_pose_client_ = create_client<TriggerT>(
    "PoseEnable", rmw_qos_profile_services_default, callback_group_);

  OnInitialize();
  points_pub_ = this->create_publisher<protocol::msg::FollowPoints>(
    "follow_points", rclcpp::SystemDefaultsQoS());

  points_subscriber_ = this->create_subscription<protocol::msg::FollowPoints>(
    "subsequent_follow_points", rclcpp::SystemDefaultsQoS(),
    std::bind(&NavigationCore::FollwPointCallback, this, _1));
  reloc_sub_ = create_subscription<std_msgs::msg::Int32>(
    "laser_reloc_result",
    rclcpp::SystemDefaultsQoS(),
    std::bind(&NavigationCore::HandleRelocCallback, this, _1));

  task_managers_ = std::make_shared<std::thread>(&NavigationCore::TaskManager, this);
}

void NavigationCore::FollwPointCallback(
  const protocol::msg::FollowPoints::SharedPtr msg)
{
  protocol::msg::FollowPoints msg_;
  int status;
  ActionType action_type;
  GetNavStatus(status, action_type);
  if ((action_type == kActionThroughPose) && (msg->poses.size() > 0)) {
    msg_.poses = msg->poses;
    msg_.token = "0xafic29casckdek";
    msg_.header = ReturnHeader();
    points_pub_->publish(msg_);
  } else {
    RCLCPP_ERROR(
      this->get_logger(),
      "action type: %d error or follow points empty", action_type);
  }
}

std_msgs::msg::Header NavigationCore::ReturnHeader()
{
  std_msgs::msg::Header msg;
  msg.frame_id = "NavigationCore";
  msg.stamp = this->get_clock()->now();
  return msg;
}

void NavigationCore::set_running_navigation(bool state)
{
  running_navigation_ = state;
}

bool NavigationCore::running_navigation()
{
  return running_navigation_;
}

void NavigationCore::set_send_result_flag(bool result)
{
  send_result_flag_ = result;
}

rclcpp_action::GoalResponse NavigationCore::HandleNavigationGoal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const Navigation::Goal> goal)
{
  (void)uuid;
  (void)goal;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse NavigationCore::HandleNavigationCancel(
  const std::shared_ptr<GoalHandleNavigation> goal_handle)
{
  INFO("Received request to cancel goal");
  (void)goal_handle;
  if (reloc_topic_waiting_) {
    INFO("notify");
    reloc_cv_.notify_one();
    reloc_topic_waiting_ = false;
  }
  OnCancel();
  return rclcpp_action::CancelResponse::ACCEPT;
}

void NavigationCore::HandleNavigationAccepted(
  const std::shared_ptr<GoalHandleNavigation> goal_handle)
{
  // this needs to return quickly to avoid blocking the executor, so spin up a
  // new thread
  goal_handle_ = goal_handle;
  std::thread{std::bind(&NavigationCore::FollowExecute, this, _1), goal_handle}
  .detach();
}

void NavigationCore::FollowExecute(
  const std::shared_ptr<GoalHandleNavigation> goal_handle)
{
  const auto goal = goal_handle->get_goal();
  auto result = std::make_shared<Navigation::Result>();
  RCLCPP_INFO(this->get_logger(), "Executing goal: %d", goal->nav_type);

  switch (goal->nav_type) {
    case Navigation::Goal::NAVIGATION_TYPE_START_AB:
      {
        INFO("robot[Navigation]  Navigation::Goal::NAVIGATION_GOAL_TYPE_AB .....");
        if (GetCurrentTaskState() == TaskState::StopNavigation) {
          WARN("Current robot stop AB point navigation, can't start navigate.");
          result->result = Navigation::Result::NAVIGATION_RESULT_TYPE_FAILED;
          goal_handle->succeed(result);
          return;
        }

        // Condition : Target pose not empty
        result->result = Navigation::Result::NAVIGATION_RESULT_TYPE_SUCCESS;
        if (goal->poses.empty()) {
          INFO("empty pose ");
          INFO("goal_handle->succeed(result) ### 4");
          goal_handle->succeed(result);
        }

        INFO(
          "Goal pose : [x = %f, y = %f]",
          goal->poses[0].pose.position.x, goal->poses[0].pose.position.y);

        // Start AB point navigation
        uint8_t goal_result = StartNavigation(goal->poses[0]);
        if (goal_result != Navigation::Result::NAVIGATION_RESULT_TYPE_ACCEPT) {
          // goal process failed
          INFO("goal_handle->succeed(result) ### 3");
          result->result = goal_result;
          goal_handle->succeed(result);
          SetTaskState(TaskState::Unknown);
        }
      }
      break;

    case Navigation::Goal::NAVIGATION_TYPE_STOP_AB:
      {
        INFO("[Navigation]  Navigation::Goal::NAVIGATION_TYPE_STOP_AB .....");
        if (GetCurrentTaskState() != TaskState::StartNavigation) {
          return;
        }

        // Wait localization has stoped state that cancel navigation AB point
        while (true) {
          if (GetCurrentTaskState() == TaskState::StopLocalization) {
            INFO("Current robot is in stop localization state");
            std::this_thread::sleep_for(std::chrono::seconds(1));
            continue;
          }

          if (GetCurrentTaskState() == TaskState::Unknown) {
            INFO("Robot stop localization success.");
            break;
          }
          std::this_thread::sleep_for(std::chrono::seconds(1));
        }

        // Cancel navigation
        bool cancel_state = CancelNavigation();
        if (cancel_state) {
          result->result = Navigation::Result::NAVIGATION_RESULT_TYPE_SUCCESS;
          set_running_navigation(false);
          INFO("Stop navigation AB point success.");
        } else {
          result->result = Navigation::Result::NAVIGATION_RESULT_TYPE_FAILED;
          WARN("Stop navigation AB point failed.");
        }

        INFO("goal_handle->succeed(result) ### 2");
        goal_handle->succeed(result);
        SetTaskState(TaskState::Unknown);
      }
      break;

    case Navigation::Goal::NAVIGATION_TYPE_START_FOLLOW:
      {
        INFO("[Navigation]  Navigation::Goal::NAVIGATION_GOAL_TYPE_FOLLOW .....");
        uint8_t goal_result = StartNavThroughPoses(goal->poses);
        if (goal_result != Navigation::Result::NAVIGATION_RESULT_TYPE_ACCEPT) {
          INFO("goal_handle->succeed(result) ### 1");
          goal_handle->succeed(result);
        }
      }
      break;

    case Navigation::Goal::NAVIGATION_TYPE_START_MAPPING:
    case Navigation::Goal::NAVIGATION_TYPE_STOP_MAPPING:
      {
        INFO("mapping request, map_name : %s", goal->map_name.c_str());
        // mapping and  localization mutex
        if (GetCurrentTaskState() == TaskState::StartLocalization) {
          INFO("Current robot in start localization state, not support start mapping.");
          return;
        }

        // mapping and  localization mutex
        if (GetCurrentTaskState() == TaskState::StopBuildMap) {
          INFO("Current robot in stop localization state, not support stop mapping.");
          return;
        }

        // Check start mapping state
        bool start = goal->nav_type == Navigation::Goal::NAVIGATION_TYPE_START_MAPPING;
        if (start) {
          if (GetCurrentTaskState() == TaskState::StartBuildMap) {
            INFO("Current robot has started building map.");
            return;
          }
        } else {
          if (GetCurrentTaskState() == TaskState::StopBuildMap) {
            INFO("Current robot has stoped building map.");
            return;
          }
        }

        // Robot start or stop mapping
        uint8_t goal_result = HandleMapping(start, goal->map_name);
        if (goal_result != Navigation::Result::NAVIGATION_RESULT_TYPE_ACCEPT) {
          INFO("[Navigation]  Navigation::Goal::NAVIGATION_GOAL_TYPE_MAPPING .....");
          result->result = Navigation::Result::NAVIGATION_RESULT_TYPE_SUCCESS;
          INFO("mapping request success");
          goal_handle->succeed(result);
        } else {
          INFO("mapping request failed");
        }
      }
      break;

    // uint8 NAVIGATION_GOAL_TYPE_LOCATION = 5
    // uint8 NAVIGATION_GOAL_TYPE_AUTO_DOCKING = 6
    case Navigation::Goal::NAVIGATION_TYPE_START_LOCALIZATION:
    case Navigation::Goal::NAVIGATION_TYPE_STOP_LOCALIZATION:
      {
        INFO("loc request");
        // mapping and  localization mutex
        if (GetCurrentTaskState() == TaskState::StopNavigation) {
          INFO("Current robot in stop navigation state, not support map localization.");
          return;
        }

        bool start = goal->nav_type == Navigation::Goal::NAVIGATION_TYPE_START_LOCALIZATION;
        // Check start mapping state
        if (start) {
          if (GetCurrentTaskState() == TaskState::StartLocalization) {
            INFO("Current robot has started localization in map.");
            return;
          }
        } else {
          if (GetCurrentTaskState() == TaskState::StopLocalization) {
            INFO("Current robot has stoped localization in map.");
            return;
          }
        }

        auto goal_result = HandleLocalization(start);
        if (goal_result == ActionExecStage::kFailed) {
          result->result = Navigation::Result::NAVIGATION_RESULT_TYPE_FAILED;
          goal_handle->abort(result);
        } else if (goal_result == ActionExecStage::kSuccess) {
          INFO("result->result = Navigation::Result::NAVIGATION_RESULT_TYPE_SUCCESS");
          result->result = Navigation::Result::NAVIGATION_RESULT_TYPE_SUCCESS;
          goal_handle->succeed(result);
        }
        SetTaskState(TaskState::Unknown);
      }
      break;

    case Navigation::Goal::NAVIGATION_TYPE_START_AUTO_DOCKING:
      {
        INFO("[Navigation] Navigation::Goal::NAVIGATION_GOAL_TYPE_AUTO_DOCKING .....");
        result->result = Navigation::Result::NAVIGATION_RESULT_TYPE_SUCCESS;
      }
      break;

    case Navigation::Goal::NAVIGATION_TYPE_START_UWB_TRACKING:
      {
        INFO("[Navigation]  Navigation::Goal::NAVIGATION_TYPE_START_WUB_TRACKING .....");
        // result->result = Navigation::Result::NAVIGATION_RESULT_TYPE_SUCCESS;
        // if (goal->poses.empty()) {
        //   INFO("empty pose ");
        //   goal_handle->succeed(result);
        // }
        uint8_t goal_result = StartTracking(goal->relative_pos, goal->keep_distance);
        if (goal_result != Navigation::Result::NAVIGATION_RESULT_TYPE_ACCEPT) {
          // goal process failed
          INFO("goal process failed");
          result->result = goal_result;
          goal_handle->succeed(result);
        }
      }
      break;

    case Navigation::Goal::NAVIGATION_TYPE_START_HUMAN_TRACKING:
      {
        INFO("[Navigation]  Navigation::Goal::NAVIGATION_TYPE_START_VISION_TRACKING .....");
        uint8_t goal_result = StartVisionTracking(goal->relative_pos, goal->keep_distance);
        if (goal_result != Navigation::Result::NAVIGATION_RESULT_TYPE_ACCEPT) {
          // goal process failed
          INFO("goal process failed");
          result->result = goal_result;
          goal_handle->succeed(result);
        }
      }
      break;
    default:
      result->result = Navigation::Result::NAVIGATION_RESULT_TYPE_REJECT;
      INFO("goal_handle->succeed(result) ### 5");
      goal_handle->succeed(result);
      break;
  }
}

bool NavigationCore::CancelNavigation()
{
  INFO("Start cancel navigation.");
  bool run_result = false;

  if (navigation_goal_handle_) {
    SetTaskState(TaskState::StopNavigation);
    auto future_cancel =
      navigation_action_client_->async_cancel_goal(navigation_goal_handle_);

    if (rclcpp::spin_until_future_complete(
        client_node_, future_cancel,
        server_timeout_) !=
      rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(client_node_->get_logger(), "Failed to cancel goal");
    } else {
      navigation_goal_handle_.reset();
      RCLCPP_ERROR(client_node_->get_logger(), "canceled navigation goal");
      run_result = true;
    }

    if (!nav_timer_->is_canceled()) {
      INFO("Canceled navigation goal timer");
      nav_timer_->cancel();
    }
  }

  return run_result;
}

bool NavigationCore::ReportRealtimeRobotPose(bool start)
{
  std::string start_cmd = "Start report robot's realtime pose.";
  std::string stop_cmd = "Stop report robot's realtime pose.";

  if (start) {
    INFO("%s", start_cmd.c_str());
  } else {
    INFO("%s", stop_cmd.c_str());
  }

  auto request = std::make_shared<std_srvs::srv::SetBool_Request>();
  request->data = start;
  INFO("realtime_pose_client_ service name: %s", realtime_pose_client_->get_service_name());
  return ServiceImpl(realtime_pose_client_, request);
}

bool NavigationCore::StopMapping(const std::string & map_saved_name)
{
  auto request = std::make_shared<visualization::srv::Stop_Request>();
  request->map_name = map_saved_name;
  request->finish = true;

  if (map_saved_name.empty()) {
    ERROR("Should set reasonable map name, must not empty");
    // return false;
    request->map_name = "map";
  }

  while (!stop_mapping_client_->wait_for_service(5s)) {
    if (!rclcpp::ok()) {
      ERROR("Interrupted while waiting for the service. Exiting.");
      return false;
    }
    WARN("service not available, waiting again...");
  }
  auto future = stop_mapping_client_->async_send_request(request);
  // Wait for the result.
  if (future.wait_for(5s) == std::future_status::timeout) {
    ERROR("Stop build mapping service timeout, and save map error.");
    return false;
  }

  INFO("Stop map building and save robot's map name : %s", request->map_name);
  return future.get()->success;
}

void NavigationCore::TaskManager()
{
  // Control task run frequency 20Hz
  constexpr int task_frequency = 20;
  rclcpp::Rate rate(task_frequency);

  while (rclcpp::ok()) {
    std::unique_lock<std::mutex> locker(navigation_mutex_);
    if (navigation_cond_.wait_for(locker, std::chrono::seconds(2)) == std::cv_status::timeout) {
      continue;
    }

    if (navigation_finished_) {
      nav_timer_->cancel();
      SenResult();
      navigation_finished_ = false;
      INFO("Start navigation AB point success.");
      SetTaskState(TaskState::Unknown);
    }

    rate.sleep();
  }
}

void NavigationCore::OnInitialize()
{
  // create action feedback subscribers
  navigation_feedback_sub_ = this->create_subscription<
    nav2_msgs::action::NavigateToPose::Impl::FeedbackMessage>(
    "navigate_to_pose/_action/feedback", rclcpp::SystemDefaultsQoS(),
    [this](const nav2_msgs::action::NavigateToPose::Impl::FeedbackMessage::
    SharedPtr msg) {
      (void)msg;
      // navigation_feedback_indicator_->setText(getNavToPoseFeedbackLabel(msg->feedback));
    });
  nav_through_poses_feedback_sub_ = this->create_subscription<
    nav2_msgs::action::NavigateThroughPoses::Impl::FeedbackMessage>(
    "navigate_through_poses/_action/feedback", rclcpp::SystemDefaultsQoS(),
    [this](const nav2_msgs::action::NavigateThroughPoses::Impl::
    FeedbackMessage::SharedPtr msg) {
      (void)msg;
      // navigation_feedback_indicator_->setText(getNavThroughPosesFeedbackLabel(msg->feedback));
    });

  // create action goal status subscribers
  navigation_goal_status_sub_ =
    this->create_subscription<action_msgs::msg::GoalStatusArray>(
    "navigate_to_pose/_action/status", rclcpp::SystemDefaultsQoS(),
    [this](const action_msgs::msg::GoalStatusArray::SharedPtr msg) {
      (void)msg;
      // navigation_goal_status_indicator_->setText(
      //   getGoalStatusLabel(msg->status_list.back().status));
      if (msg->status_list.back().status !=
      action_msgs::msg::GoalStatus::STATUS_EXECUTING)
      {
        // navigation_feedback_indicator_->setText(getNavToPoseFeedbackLabel());
      }
    });

  nav_through_poses_goal_status_sub_ =
    this->create_subscription<action_msgs::msg::GoalStatusArray>(
    "navigate_through_poses/_action/status", rclcpp::SystemDefaultsQoS(),
    [this](const action_msgs::msg::GoalStatusArray::SharedPtr msg) {
      (void)msg;
      // navigation_goal_status_indicator_->setText(
      //   getGoalStatusLabel(msg->status_list.back().status));
      if (msg->status_list.back().status !=
      action_msgs::msg::GoalStatus::STATUS_EXECUTING)
      {
        // navigation_feedback_indicator_->setText(getNavThroughPosesFeedbackLabel());
      }
    });
}

uint8_t NavigationCore::StartNavigation(geometry_msgs::msg::PoseStamped pose)
{
  // start navigation stack
  if (client_nav_.is_active() != nav2_lifecycle_manager::SystemStatus::ACTIVE) {
    if (!client_nav_.startup()) {
      WARN("navigation client startup failed.");
      return Navigation::Result::NAVIGATION_RESULT_TYPE_FAILED;
    }
  }

  auto is_action_server_ready =
    navigation_action_client_->wait_for_action_server(
    std::chrono::seconds(5));
  if (!is_action_server_ready) {
    RCLCPP_ERROR(
      client_node_->get_logger(),
      "navigate_to_pose action server is not available."
      " Is the initial pose set?");
    return Navigation::Result::NAVIGATION_RESULT_TYPE_UNAVALIBLE;
    client_nav_.pause();
  }

  // Send the goal pose
  navigation_goal_.pose = pose;
  navigation_goal_.pose.header.frame_id = "map";
  navigation_goal_.pose.pose.orientation.w = 1;

  INFO("NavigateToPose will be called using the BT Navigator's default behavior tree.");

  // Enable result awareness by providing an empty lambda function
  auto send_goal_options = rclcpp_action::Client<
    nav2_msgs::action::NavigateToPose>::SendGoalOptions();

  send_goal_options.result_callback = [this](auto) {
      ERROR("Get navigate to poses result");
      SetTaskState(TaskState::Unknown);
      // SenResult();
      // navigation_goal_handle_.reset();
    };

  auto future_goal_handle = navigation_action_client_->async_send_goal(
    navigation_goal_, send_goal_options);
  if (rclcpp::spin_until_future_complete(
      client_node_, future_goal_handle,
      server_timeout_) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    ERROR("Send goal call failed");
    client_nav_.pause();
    return Navigation::Result::NAVIGATION_RESULT_TYPE_FAILED;
  }

  if (future_goal_handle.wait_for(server_timeout_) == std::future_status::ready) {
    INFO("Send goal success.");
    // Get the goal handle and save so that we can check on completion in the
    // timer callback
    navigation_goal_handle_ = future_goal_handle.get();
    if (!navigation_goal_handle_) {
      RCLCPP_ERROR(client_node_->get_logger(), "Goal was rejected by server");
      client_nav_.pause();
      return Navigation::Result::NAVIGATION_RESULT_TYPE_REJECT;
    }

    nav_timer_ = this->create_wall_timer(
      200ms, std::bind(&NavigationCore::NavigationStatusFeedbackMonitor, this));

    SetTaskState(TaskState::StartNavigation);
    return Navigation::Result::NAVIGATION_RESULT_TYPE_ACCEPT;
  }

  ERROR("Send goal call failed");
  return Navigation::Result::NAVIGATION_RESULT_TYPE_FAILED;
}

uint8_t NavigationCore::StartTracking(uint8_t relative_pos, float keep_distance)
{
  // start navigation stack
  if (client_nav_.is_active() != nav2_lifecycle_manager::SystemStatus::ACTIVE) {
    if (!client_nav_.startup()) {
      return Navigation::Result::NAVIGATION_RESULT_TYPE_FAILED;
    }
  }
  if (client_mcr_uwb_.is_active() != nav2_lifecycle_manager::SystemStatus::ACTIVE) {
    if (!client_mcr_uwb_.startup()) {
      return Navigation::Result::NAVIGATION_RESULT_TYPE_FAILED;
    }
  }
  auto is_action_server_ready =
    target_tracking_action_client_->wait_for_action_server(
    std::chrono::seconds(5));
  if (!is_action_server_ready) {
    ERROR("Tracking target action server is not available.");
    return Navigation::Result::NAVIGATION_RESULT_TYPE_UNAVALIBLE;
    client_nav_.pause();
    client_mcr_uwb_.pause();
  }

  // Send the goal pose
  // navigation_goal_.pose = pose;
  target_tracking_goal_.keep_distance = keep_distance;
  target_tracking_goal_.relative_pos = relative_pos;
  // INFO("NavigateToPose will be called using the BT Navigator's default behavior tree.");

  // Enable result awareness by providing an empty lambda function
  auto send_goal_options = rclcpp_action::Client<
    mcr_msgs::action::TargetTracking>::SendGoalOptions();
  send_goal_options.result_callback = [this](auto) {
      ERROR("Get navigate to poses result");
      SenResult();
      target_tracking_goal_handle_.reset();
    };

  auto future_goal_handle = target_tracking_action_client_->async_send_goal(
    target_tracking_goal_, send_goal_options);
  if (rclcpp::spin_until_future_complete(
      client_node_, future_goal_handle,
      server_timeout_) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    ERROR("Send goal call failed");
    client_nav_.pause();
    client_mcr_uwb_.pause();
    return Navigation::Result::NAVIGATION_RESULT_TYPE_FAILED;
  }

  // Get the goal handle and save so that we can check on completion in the
  // timer callback
  target_tracking_goal_handle_ = future_goal_handle.get();
  if (!target_tracking_goal_handle_) {
    RCLCPP_ERROR(client_node_->get_logger(), "Goal was rejected by server");
    client_nav_.pause();
    client_mcr_uwb_.pause();
    return Navigation::Result::NAVIGATION_RESULT_TYPE_REJECT;
  }

  // nav_timer_ = this->create_wall_timer(
  //   200ms, std::bind(&NavigationCore::GetCurrentNavStatus, this));
  nav_timer_ = this->create_wall_timer(
    200ms, std::bind(&NavigationCore::NavigationStatusFeedbackMonitor, this));
  return Navigation::Result::NAVIGATION_RESULT_TYPE_ACCEPT;
}


uint8_t NavigationCore::HandleMapping(bool start, const std::string & map_saved_name)
{
  INFO("HandleMapping:  %s", start ? "start" : "stop");
  auto request = std::make_shared<std_srvs::srv::SetBool_Request>();
  request->data = true;
  rclcpp::Client<TriggerT>::SharedPtr client;

  if (start) {
    // real sense
    // if (!client_realsense_->Startup()) {
    //   ERROR("Realsense lifecycle start failed");
    //   return Navigation::Result::NAVIGATION_RESULT_TYPE_FAILED;
    // }
    SetTaskState(TaskState::StartBuildMap);

    if (!realsense_lifecycle_controller_->Startup()) {
      ERROR("Realsense lifecycle start failed");
      return Navigation::Result::NAVIGATION_RESULT_TYPE_FAILED;
    }

    if (client_mapping_.is_active() != nav2_lifecycle_manager::SystemStatus::ACTIVE) {
      if (!client_mapping_.startup()) {
        ERROR("Lifecycle start failed");
        SetTaskState(TaskState::Unknown);
        return Navigation::Result::NAVIGATION_RESULT_TYPE_FAILED;
      }
    }

    client = start_mapping_client_;
    if (!ServiceImpl(client, request)) {
      ERROR("Service failed");
      SetTaskState(TaskState::Unknown);
      return Navigation::Result::NAVIGATION_RESULT_TYPE_FAILED;
    }

    if (!ReportRealtimeRobotPose(start)) {
      INFO("Start robot's realtime pose failed.");
      SetTaskState(TaskState::Unknown);
      return Navigation::Result::NAVIGATION_RESULT_TYPE_FAILED;
    }
  } else {
    SetTaskState(TaskState::StartBuildMap);

    if (client_mapping_.is_active() != nav2_lifecycle_manager::SystemStatus::ACTIVE) {
      INFO("Failed to stop mapping because node not active");
      SetTaskState(TaskState::Unknown);
      return Navigation::Result::NAVIGATION_RESULT_TYPE_FAILED;
    }

    if (!StopMapping(map_saved_name)) {
      ERROR("stop mapping service failed");
      SetTaskState(TaskState::Unknown);
      return Navigation::Result::NAVIGATION_RESULT_TYPE_FAILED;
    }

    if (!client_mapping_.pause()) {
      ERROR("Lifecycle pause failed");
      SetTaskState(TaskState::Unknown);
      return Navigation::Result::NAVIGATION_RESULT_TYPE_FAILED;
    }

    if (!realsense_lifecycle_controller_->Pause()) {
      ERROR("Realsense lifecycle pause failed");
      SetTaskState(TaskState::Unknown);
      return Navigation::Result::NAVIGATION_RESULT_TYPE_FAILED;
    }

    if (!ReportRealtimeRobotPose(start)) {
      INFO("Stop robot's realtime pose failed.");
      SetTaskState(TaskState::Unknown);
      return Navigation::Result::NAVIGATION_RESULT_TYPE_FAILED;
    }
  }
  INFO("Call HandleMapping() function success.");
  return Navigation::Result::NAVIGATION_RESULT_TYPE_SUCCESS;
}

ActionExecStage NavigationCore::HandleLocalization(bool start)
{
  INFO("HandleLocalization:  %s", start ? "start" : "stop");
  auto request = std::make_shared<std_srvs::srv::SetBool_Request>();
  request->data = true;
  rclcpp::Client<TriggerT>::SharedPtr client;

  if (start) {
    // real sense
    if (!realsense_lifecycle_controller_->Startup()) {
      ERROR("Realsense lifecycle start failed");
      return ActionExecStage::kFailed;
    }

    if (client_loc_.is_active() != nav2_lifecycle_manager::SystemStatus::ACTIVE) {
      if (!client_loc_.startup()) {
        ERROR("Localization lifecycle start failed");
        SetTaskState(TaskState::Unknown);
        return ActionExecStage::kFailed;
      }
    }

    client = start_loc_client_;
    if (!ServiceImpl(client, request)) {
      ERROR("Service failed");
      SetTaskState(TaskState::Unknown);
      return ActionExecStage::kFailed;
    }

    if (!ReportRealtimeRobotPose(start)) {
      ERROR("Start report robot's realtime pose failed.");
      SetTaskState(TaskState::Unknown);
      return ActionExecStage::kFailed;
    }

    std::thread{std::bind(&NavigationCore::GetCurrentLocStatus, this)}.detach();
    SetTaskState(TaskState::StartLocalization);
    return ActionExecStage::kExecuting;
  } else {
    // Set state flag
    SetTaskState(TaskState::StopLocalization);
    if (client_loc_.is_active() != nav2_lifecycle_manager::SystemStatus::ACTIVE) {
      INFO("Failed to stop mapping because node not active");
      return ActionExecStage::kFailed;
    }

    client = stop_loc_client_;
    if (!ServiceImpl(client, request)) {
      ERROR("Service failed");
      SetTaskState(TaskState::Unknown);
      return ActionExecStage::kFailed;
    }

    if (!client_loc_.pause()) {
      ERROR("Localization lifecycle pause failed");
      SetTaskState(TaskState::Unknown);
      return ActionExecStage::kFailed;
    }

    if (!realsense_lifecycle_controller_->Pause()) {
      ERROR("Realsense lifecycle pause failed");
      SetTaskState(TaskState::Unknown);
      return ActionExecStage::kFailed;
    }

    if (!ReportRealtimeRobotPose(start)) {
      ERROR("Stop report robot's realtime pose failed.");
      SetTaskState(TaskState::Unknown);
      return ActionExecStage::kFailed;
    }

    INFO("Stop lidar slam localization success.");
    reloc_status_ = RelocStatus::kIdle;
    return ActionExecStage::kSuccess;
  }
}

uint8_t NavigationCore::HandleVisionMapping(bool start)
{
  // Check start mapping state
  if (start) {
    if (GetCurrentTaskState() == TaskState::StartBuildMap) {
      INFO("Current robot has start building map.");
      return Navigation::Result::NAVIGATION_RESULT_TYPE_SUCCESS;
    }
  } else {
    if (GetCurrentTaskState() == TaskState::StopBuildMap) {
      INFO("Current robot has stop building map.");
      return Navigation::Result::NAVIGATION_RESULT_TYPE_SUCCESS;
    }
  }

  INFO("HandleVisionMapping:  %s", start ? "start" : "stop");

  if (start) {
    // Turn on realsense sensor
    if (!realsense_lifecycle_controller_->Startup()) {
      ERROR("Realsense lifecycle start failed");
      return Navigation::Result::NAVIGATION_RESULT_TYPE_FAILED;
    }

    // Turn on R-GBD camera

    // Startup vision mapping
    if (!vision_mapping_lifecycle_controller_->Startup()) {
      ERROR("Startup vision mapping lifecycle failed");
      SetTaskState(TaskState::Unknown);
      return Navigation::Result::NAVIGATION_RESULT_TYPE_FAILED;
    }

    if (!ReportRealtimeRobotPose(start)) {
      INFO("Start robot's realtime pose failed.");
      return Navigation::Result::NAVIGATION_RESULT_TYPE_FAILED;
    }

    INFO("Start vision mapping success.");
    SetTaskState(TaskState::StartBuildMap);
  } else {
    if (!vision_mapping_lifecycle_controller_->Pause()) {
      ERROR("Pause vision mapping lifecycle failed");
      return Navigation::Result::NAVIGATION_RESULT_TYPE_FAILED;
    }

    if (!StopMapping("map")) {
      ERROR("stop mapping service failed");
      SetTaskState(TaskState::Unknown);
      return Navigation::Result::NAVIGATION_RESULT_TYPE_FAILED;
    }

    if (!realsense_lifecycle_controller_->Pause()) {
      ERROR("Realsense lifecycle pause failed");
      return Navigation::Result::NAVIGATION_RESULT_TYPE_FAILED;
    }

    if (!ReportRealtimeRobotPose(start)) {
      INFO("Stop robot's realtime pose failed.");
      return Navigation::Result::NAVIGATION_RESULT_TYPE_FAILED;
    }

    SetTaskState(TaskState::StartBuildMap);
    INFO("Stop vision mapping success.");
  }
  return Navigation::Result::NAVIGATION_RESULT_TYPE_SUCCESS;
}

ActionExecStage NavigationCore::HandleVisionLocalization(bool start)
{
  return ActionExecStage::kSuccess;
}

uint8_t NavigationCore::HandleMapping(bool start, bool outdoor)
{
  if (outdoor) {
    INFO("Choose outdoor mapping and lidar mapping.");
    // return HandleMapping(start);
  }

  INFO("Choose indoor mapping and vision mapping.");
  return HandleVisionMapping(start);
}

bool NavigationCore::ServiceImpl(
  const rclcpp::Client<TriggerT>::SharedPtr client,
  const std_srvs::srv::SetBool_Request::SharedPtr request)
{
  while (!client->wait_for_service(5s)) {
    if (!rclcpp::ok()) {
      ERROR("Interrupted while waiting for the service. Exiting.");
      return false;
    }
    WARN("service not available, waiting again...");
  }
  auto future = client->async_send_request(request);
  // Wait for the result.
  if (future.wait_for(5s) == std::future_status::timeout) {
    ERROR("Service timeout");
    return false;
  }

  return future.get()->success;
}

uint8_t NavigationCore::StartNavThroughPoses(
  std::vector<geometry_msgs::msg::PoseStamped> poses)
{
  auto is_action_server_ready =
    nav_through_poses_action_client_->wait_for_action_server(
    std::chrono::seconds(5));
  if (!is_action_server_ready) {
    RCLCPP_ERROR(
      client_node_->get_logger(),
      "navigate_through_poses action server is not available."
      " Is the initial pose set?");
    return Navigation::Result::NAVIGATION_RESULT_TYPE_UNAVALIBLE;
  }

  nav_through_poses_goal_.poses = poses;
  RCLCPP_INFO(
    client_node_->get_logger(),
    "NavigateThroughPoses will be called using the BT Navigator's "
    "default behavior tree.");

  RCLCPP_DEBUG(
    client_node_->get_logger(), "Sending a path of %zu waypoints:",
    nav_through_poses_goal_.poses.size());
  for (auto waypoint : nav_through_poses_goal_.poses) {
    RCLCPP_DEBUG(
      client_node_->get_logger(), "\t(%lf, %lf)",
      waypoint.pose.position.x, waypoint.pose.position.y);
  }

  // Enable result awareness by providing an empty lambda function
  auto send_goal_options = rclcpp_action::Client<
    nav2_msgs::action::NavigateThroughPoses>::SendGoalOptions();
  send_goal_options.result_callback = [this](auto) {
      RCLCPP_ERROR(
        client_node_->get_logger(),
        "Get navigate_through_poses result");
      nav_through_poses_goal_handle_.reset();
      SenResult();
    };

  auto future_goal_handle = nav_through_poses_action_client_->async_send_goal(
    nav_through_poses_goal_, send_goal_options);
  if (rclcpp::spin_until_future_complete(
      client_node_, future_goal_handle,
      server_timeout_) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(client_node_->get_logger(), "Send goal call failed");
    return Navigation::Result::NAVIGATION_RESULT_TYPE_FAILED;
  }

  // Get the goal handle and save so that we can check on completion in the
  // timer callback
  nav_through_poses_goal_handle_ = future_goal_handle.get();
  if (!nav_through_poses_goal_handle_) {
    RCLCPP_ERROR(client_node_->get_logger(), "Goal was rejected by server");
    return Navigation::Result::NAVIGATION_RESULT_TYPE_REJECT;
  }

  through_pose_timer_ = this->create_wall_timer(
    200ms, std::bind(&NavigationCore::NavigationStatusFeedbackMonitor, this));

  return Navigation::Result::NAVIGATION_RESULT_TYPE_ACCEPT;
}

uint8_t NavigationCore::StartWaypointFollowing(
  std::vector<geometry_msgs::msg::PoseStamped> poses)
{
  auto is_action_server_ready =
    waypoint_follower_action_client_->wait_for_action_server(
    std::chrono::seconds(5));
  if (!is_action_server_ready) {
    RCLCPP_ERROR(
      client_node_->get_logger(),
      "follow_waypoints action server is not available."
      " Is the initial pose set?");
    return Navigation::Result::NAVIGATION_RESULT_TYPE_UNAVALIBLE;
  }

  // Send the goal poses
  waypoint_follower_goal_.poses = poses;

  RCLCPP_DEBUG(
    client_node_->get_logger(), "Sending a path of %zu waypoints:",
    waypoint_follower_goal_.poses.size());
  for (auto waypoint : waypoint_follower_goal_.poses) {
    RCLCPP_DEBUG(
      client_node_->get_logger(), "\t(%lf, %lf)",
      waypoint.pose.position.x, waypoint.pose.position.y);
  }

  // Enable result awareness by providing an empty lambda function
  auto send_goal_options = rclcpp_action::Client<
    nav2_msgs::action::FollowWaypoints>::SendGoalOptions();
  send_goal_options.result_callback = [this](auto) {
      RCLCPP_ERROR(client_node_->get_logger(), "Get follow waypoints result");
      SenResult();
      waypoint_follower_goal_handle_.reset();
    };

  auto future_goal_handle = waypoint_follower_action_client_->async_send_goal(
    waypoint_follower_goal_, send_goal_options);
  if (rclcpp::spin_until_future_complete(
      client_node_, future_goal_handle,
      server_timeout_) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(client_node_->get_logger(), "Send goal call failed");
    return Navigation::Result::NAVIGATION_RESULT_TYPE_FAILED;
  }

  // Get the goal handle and save so that we can check on completion in the
  // timer callback
  waypoint_follower_goal_handle_ = future_goal_handle.get();
  if (!waypoint_follower_goal_handle_) {
    RCLCPP_ERROR(client_node_->get_logger(), "Goal was rejected by server");
    return Navigation::Result::NAVIGATION_RESULT_TYPE_REJECT;
  }

  waypoint_follow_timer_ = this->create_wall_timer(
    200ms, std::bind(&NavigationCore::NavigationStatusFeedbackMonitor, this));
  return Navigation::Result::NAVIGATION_RESULT_TYPE_ACCEPT;
}

void NavigationCore::GetNavStatus(int & status, ActionType & action_type)
{
  status = status_;
  action_type = action_type_;
}

void NavigationCore::NavigationStatusFeedbackMonitor()
{
  // RCLCPP_INFO(client_node_->get_logger(), "Navigation status monitor ...");
  static auto feedback = std::make_shared<Navigation::Feedback>();
  if (start_vision_tracking_) {
    rclcpp::spin_some(client_node_);
    {
      feedback->feedback_code = vision_action_client_feedback_;
      goal_handle_->publish_feedback(feedback);
      // state_machine_.postEvent(new ROSActionQEvent(QActionState::ACTIVE));
    }
    return;
  }
  if (!waypoint_follower_goal_handle_ && !nav_through_poses_goal_handle_ &&
    !navigation_goal_handle_)
  {
    // RCLCPP_ERROR(client_node_->get_logger(), "Waiting for Goal");
    // state_machine_.postEvent(new ROSActionQEvent(QActionState::INACTIVE));
    action_type_ = kActionNone;
    return;
  } else if (waypoint_follower_goal_handle_) {
    rclcpp::spin_some(client_node_);
    status_ = waypoint_follower_goal_handle_->get_status();
    action_type_ = kActionWayPoint;
    // Check if the goal is still executing
    if (status_ == GoalStatus::STATUS_ACCEPTED ||
      status_ == GoalStatus::STATUS_EXECUTING)
    {
      // state_machine_.postEvent(new ROSActionQEvent(QActionState::ACTIVE));
    } else {
      // state_machine_.postEvent(new ROSActionQEvent(QActionState::INACTIVE));
      waypoint_follow_timer_->cancel();
      RCLCPP_ERROR(client_node_->get_logger(), "Way point follow finished");
      SenResult();
    }
  } else if (nav_through_poses_goal_handle_) {
    rclcpp::spin_some(client_node_);
    status_ = nav_through_poses_goal_handle_->get_status();
    action_type_ = kActionThroughPose;
    // Check if the goal is still executing
    if (status_ == GoalStatus::STATUS_ACCEPTED ||
      status_ == GoalStatus::STATUS_EXECUTING)
    {
      // state_machine_.postEvent(new ROSActionQEvent(QActionState::ACTIVE));
    } else {
      // state_machine_.postEvent(new ROSActionQEvent(QActionState::INACTIVE));
      RCLCPP_ERROR(client_node_->get_logger(), "through poses finished");
      through_pose_timer_->cancel();
      SenResult();
    }
  } else if (navigation_goal_handle_) {
    // rclcpp::spin_some(client_node_);
    status_ = navigation_goal_handle_->get_status();
    action_type_ = kActionNavigation;
    // Check if the goal is still executing
    if (status_ == GoalStatus::STATUS_ACCEPTED ||
      status_ == GoalStatus::STATUS_EXECUTING)
    {
      feedback->feedback_code = Navigation::Feedback::NAVIGATION_FEEDBACK_NAVIGATING_AB;
      goal_handle_->publish_feedback(feedback);
      // state_machine_.postEvent(new ROSActionQEvent(QActionState::ACTIVE));
    } else {
      // state_machine_.postEvent(new ROSActionQEvent(QActionState::INACTIVE));
      RCLCPP_ERROR(client_node_->get_logger(), "navigation to pose finished");
      // nav_timer_->cancel();
      // navigation_finished_ = false;
      navigation_finished_ = true;
      navigation_cond_.notify_one();
      navigation_goal_handle_.reset();
    }
  }
}

void NavigationCore::GetCurrentLocStatus()
{
  while (rclcpp::ok()) {
    reloc_topic_waiting_ = true;
    std::unique_lock<std::mutex> lk(reloc_mutex_);
    if (reloc_cv_.wait_for(lk, std::chrono::seconds(20)) == std::cv_status::timeout) {
      auto result = std::make_shared<Navigation::Result>();
      result->result = Navigation::Result::NAVIGATION_RESULT_TYPE_FAILED;
      // goal_handle_->abort(result);
      ResetReloc();
      ERROR("Get localization topic result timeout");
      return;
    }

    if (goal_handle_->is_canceling()) {
      auto result = std::make_shared<Navigation::Result>();
      result->result = Navigation::Result::NAVIGATION_RESULT_TYPE_CANCEL;
      goal_handle_->canceled(result);
      reloc_status_ = RelocStatus::kIdle;
      return;
    }

    static auto feedback = std::make_shared<Navigation::Feedback>();
    if (reloc_status_ == RelocStatus::kRetrying) {
      feedback->feedback_code = static_cast<int32_t>(reloc_status_);
      goal_handle_->publish_feedback(feedback);
      INFO("feeding back");
      // std::this_thread::sleep_for(std::chrono::milliseconds(20));
      return;
    }

    if (reloc_status_ == RelocStatus::kFailed) {
      feedback->feedback_code = static_cast<int32_t>(reloc_status_);
      auto result = std::make_shared<Navigation::Result>();
      result->result = Navigation::Result::NAVIGATION_RESULT_TYPE_FAILED;
      // goal_handle_->abort(result);
      ResetReloc();
      return;
    }

    if (reloc_status_ == RelocStatus::kSuccess ||
      GetCurrentTaskState() == TaskState::StartLocalization)
    {
      SenResult();
      reloc_status_ = RelocStatus::kIdle;
      return;
    }
  }
}

void NavigationCore::SenResult()
{
  auto result = std::make_shared<Navigation::Result>();
  result->result = Navigation::Result::NAVIGATION_RESULT_TYPE_SUCCESS;

  INFO("goal_handle->succeed(result) ### 6");
  goal_handle_->succeed(result);
  set_send_result_flag(true);

  action_type_ = kActionNone;
}
void NavigationCore::OnCancel()
{
  if (start_vision_tracking_) {
    if ((!client_realsense_manager_->change_state(
        lifecycle_msgs::msg::Transition::
        TRANSITION_DEACTIVATE)))
    {
      ERROR("realsense_manager lifecycle TRANSITION_DEACTIVATE failed");
    }
    if ((!client_vision_manager_->change_state(
        lifecycle_msgs::msg::Transition::
        TRANSITION_DEACTIVATE)))
    {
      ERROR("vision_manager lifecycle TRANSITION_DEACTIVATE failed");
    }
    if (target_tracking_goal_handle_) {
      if (!client_tracking_manager_->change_state(
          lifecycle_msgs::msg::Transition::
          TRANSITION_DEACTIVATE))
      {
        ERROR("tracking_manager_ lifecycle TRANSITION_DEACTIVATE failed");
      }
      auto future_cancel =
        target_tracking_action_client_->async_cancel_goal(target_tracking_goal_handle_);

      if (rclcpp::spin_until_future_complete(
          client_node_, future_cancel,
          server_timeout_) !=
        rclcpp::FutureReturnCode::SUCCESS)
      {
        RCLCPP_ERROR(client_node_->get_logger(), "Failed to cancel goal");
      } else {
        // target_tracking_action_client_.reset();
        RCLCPP_INFO(client_node_->get_logger(), "canceled navigation goal");
      }
      client_nav_.pause();
    }
    start_vision_tracking_ = false;
    return;
  }
  if (!waypoint_follower_goal_handle_ && !nav_through_poses_goal_handle_ &&
    !navigation_goal_handle_ && reloc_status_ != RelocStatus::kRetrying &&
    !target_tracking_goal_handle_)
  {
    ERROR("nothing to cancel");
  }
  if (reloc_status_ == RelocStatus::kRetrying) {
    ResetReloc();
  }

  if (navigation_goal_handle_) {
    auto future_cancel =
      navigation_action_client_->async_cancel_goal(navigation_goal_handle_);

    if (rclcpp::spin_until_future_complete(
        client_node_, future_cancel,
        server_timeout_) !=
      rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(client_node_->get_logger(), "Failed to cancel goal");
    } else {
      navigation_goal_handle_.reset();
      RCLCPP_ERROR(client_node_->get_logger(), "canceled navigation goal");
    }

    if (future_cancel.wait_for(server_timeout_) == std::future_status::ready) {
      INFO("Cancel navigation goal success.");
    } else {
      INFO("Cancel navigation goal failure.");
    }

    if (!nav_timer_->is_canceled()) {
      INFO("canceled navigation goal timer");
      nav_timer_->cancel();
    }
  }

  if (waypoint_follower_goal_handle_) {
    auto future_cancel = waypoint_follower_action_client_->async_cancel_goal(
      waypoint_follower_goal_handle_);

    if (rclcpp::spin_until_future_complete(
        client_node_, future_cancel,
        server_timeout_) !=
      rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(
        client_node_->get_logger(),
        "Failed to cancel waypoint follower");
    } else {
      waypoint_follower_goal_handle_.reset();
      RCLCPP_ERROR(client_node_->get_logger(), "canceled waypoint follower");
    }
    waypoint_follow_timer_->cancel();
  }

  if (nav_through_poses_goal_handle_) {
    auto future_cancel = nav_through_poses_action_client_->async_cancel_goal(
      nav_through_poses_goal_handle_);

    if (rclcpp::spin_until_future_complete(
        client_node_, future_cancel,
        server_timeout_) !=
      rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(
        client_node_->get_logger(),
        "Failed to cancel nav through pose action");
    } else {
      nav_through_poses_goal_handle_.reset();
      RCLCPP_ERROR(
        client_node_->get_logger(),
        "canceled nav through pose action");
    }
    through_pose_timer_->cancel();
  }

  if (target_tracking_goal_handle_) {
    auto future_cancel =
      target_tracking_action_client_->async_cancel_goal(target_tracking_goal_handle_);

    if (rclcpp::spin_until_future_complete(
        client_node_, future_cancel,
        server_timeout_) !=
      rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(client_node_->get_logger(), "Failed to cancel goal");
    } else {
      target_tracking_action_client_.reset();
      RCLCPP_ERROR(client_node_->get_logger(), "canceled navigation goal");
    }
    client_nav_.pause();
    client_mcr_uwb_.pause();
  }
  // auto result = std::make_shared<Navigation::Result>();
  // result->result = Navigation::Result::NAVIGATION_RESULT_TYPE_CANCEL;
  // goal_handle_->canceled(result);
}

TaskState NavigationCore::GetCurrentTaskState()
{
  return task_state_;
}

void NavigationCore::SetTaskState(const TaskState & state)
{
  std::lock_guard<std::mutex> locker(state_mutex_);
  task_state_ = state;
}

std::string NavigationCore::ToString(int type)
{
  std::string message;
  switch (type) {
    case Navigation::Goal::NAVIGATION_TYPE_START_AB:
      message = "NAVIGATION_TYPE_START_AB";
      break;

    case Navigation::Goal::NAVIGATION_TYPE_STOP_AB:
      message = "NAVIGATION_TYPE_STOP_AB";
      break;

    case Navigation::Goal::NAVIGATION_TYPE_START_MAPPING:
      message = "NAVIGATION_TYPE_START_MAPPING";
      break;

    case Navigation::Goal::NAVIGATION_TYPE_STOP_MAPPING:
      message = "NAVIGATION_TYPE_STOP_MAPPING";
      break;

    case Navigation::Goal::NAVIGATION_TYPE_START_LOCALIZATION:
      message = "NAVIGATION_TYPE_START_LOCALIZATION";
      break;

    case Navigation::Goal::NAVIGATION_TYPE_STOP_LOCALIZATION:
      message = "NAVIGATION_TYPE_STOP_LOCALIZATION";
      break;

    default:
      break;
  }

  return message;
}
}  // namespace carpo_navigation
