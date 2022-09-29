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

#ifndef MOTION_CORE__MOTION_CORE_NODE_HPP_
#define MOTION_CORE__MOTION_CORE_NODE_HPP_

#include <memory>
#include <string>
#include <vector>
#include <deque>

#include "nav2_lifecycle_manager/lifecycle_manager_client.hpp"
#include "nav2_msgs/action/follow_waypoints.hpp"
#include "nav2_msgs/action/navigate_through_poses.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "mcr_msgs/action/target_tracking.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "protocol/action/navigation.hpp"
#include "protocol/msg/follow_points.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_msgs/msg/int32.hpp"
#include "cyberdog_common/cyberdog_log.hpp"
#include "motion_core/realsense_lifecycle_manager.hpp"
#include "visualization/srv/stop.hpp"
// TODO(PDF)
#include "protocol/srv/body_region.hpp"
#include "protocol/srv/algo_manager.hpp"
#include "nav2_util/lifecycle_service_client.hpp"
enum ActionType
{
  kActionNone,
  kActionNavigation,
  kActionWayPoint,
  kActionThroughPose,
};

enum class ActionExecStage : uint8_t
{
  kExecuting,
  kSuccess,
  kFailed,
};

enum class RelocStatus : int32_t
{
  kIdle = -1,
  kSuccess = 0,
  kRetrying = 100,
  kFailed = 200
};

enum class TaskTypeWWW
{
  Unknown,
  StartBuildMap,
  StopBuildMap,
  StartLocalization,
  StopLocalization,
  StartNavigation,
  StopNavigation
};
namespace carpo_navigation
{
using std::placeholders::_1;
using std::placeholders::_2;
class NavigationCore : public rclcpp::Node
{
public:
  using NavigationGoalHandle =
    rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>;
  using WaypointFollowerGoalHandle =
    rclcpp_action::ClientGoalHandle<nav2_msgs::action::FollowWaypoints>;
  using NavThroughPosesGoalHandle =
    rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateThroughPoses>;
  using TargetTrackingGoalHandle =
    rclcpp_action::ClientGoalHandle<mcr_msgs::action::TargetTracking>;
  using VisionTrackingGoalHandle =
    rclcpp_action::ClientGoalHandle<mcr_msgs::action::TargetTracking>;

  using GoalStatus = action_msgs::msg::GoalStatus;
  using Navigation = protocol::action::Navigation;
  using GoalHandleNavigation = rclcpp_action::ServerGoalHandle<Navigation>;
  using TriggerT = std_srvs::srv::SetBool;
  using RealSenseClient = RealSenseLifecycleServiceClient;
  // TODO(PDF):
  using BodyRegionT = protocol::srv::BodyRegion;

  NavigationCore();
  ~NavigationCore() = default;

public:
  void OnInitialize();

  // start a2b navigation
  uint8_t StartNavigation(geometry_msgs::msg::PoseStamped pose);

  // start a2b navigation
  uint8_t StartTracking(uint8_t relative_pos, float keep_distance);

  // cancel a navigation request
  void OnCancel();

  // way point following
  uint8_t StartWaypointFollowing(
    std::vector<geometry_msgs::msg::PoseStamped> poses);

  // start through poses
  uint8_t StartNavThroughPoses(
    std::vector<geometry_msgs::msg::PoseStamped> poses);

  // get current navstatus.
  void NavigationStatusFeedbackMonitor();

  void GetCurrentLocStatus();

  // mapping
  uint8_t HandleMapping(bool start);
  ActionExecStage HandleLocalization(bool start);
  // TODO(PDF)
  void TrackingSrv_callback(
    const std::shared_ptr<rmw_request_id_t>,
    const std::shared_ptr<BodyRegionT::Request> req,
    std::shared_ptr<BodyRegionT::Response> res);

  uint8_t StartVisionTracking(uint8_t relative_pos, float keep_distance);

  std::shared_ptr<RealSenseClient> GetRealSenseNode() {return client_realsense_;}

  std::shared_ptr<rclcpp::Node> GetClientNode() {return client_node_;}

  bool TrackingClient_call_service(
  rclcpp::Client<protocol::srv::BodyRegion>::SharedPtr & client,
  const sensor_msgs::msg::RegionOfInterest & roi);

  void CallVisionTrackAlgo();
private:
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr
    navigation_action_client_;
  rclcpp_action::Client<nav2_msgs::action::FollowWaypoints>::SharedPtr
    waypoint_follower_action_client_;
  rclcpp_action::Client<nav2_msgs::action::NavigateThroughPoses>::SharedPtr
    nav_through_poses_action_client_;
  rclcpp_action::Client<mcr_msgs::action::TargetTracking>::SharedPtr
    target_tracking_action_client_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr reloc_sub_;

  // Navigation action feedback subscribers
  rclcpp::Subscription<
    nav2_msgs::action::NavigateToPose::Impl::FeedbackMessage>::SharedPtr
    navigation_feedback_sub_;

  rclcpp::Subscription<
    nav2_msgs::action::NavigateThroughPoses::Impl::FeedbackMessage>::SharedPtr
    nav_through_poses_feedback_sub_;

  rclcpp::Subscription<
    nav2_msgs::action::NavigateToPose::Impl::GoalStatusMessage>::SharedPtr
    navigation_goal_status_sub_;

  rclcpp::Subscription<
    nav2_msgs::action::NavigateThroughPoses::Impl::GoalStatusMessage>::
  SharedPtr nav_through_poses_goal_status_sub_;

  // Goal-related state
  nav2_msgs::action::NavigateToPose::Goal navigation_goal_;
  nav2_msgs::action::FollowWaypoints::Goal waypoint_follower_goal_;
  nav2_msgs::action::NavigateThroughPoses::Goal nav_through_poses_goal_;
  mcr_msgs::action::TargetTracking::Goal target_tracking_goal_;

  rclcpp::Node::SharedPtr client_node_;
  std::chrono::milliseconds server_timeout_;

  // Goal handlers
  NavigationGoalHandle::SharedPtr navigation_goal_handle_;
  WaypointFollowerGoalHandle::SharedPtr waypoint_follower_goal_handle_;
  NavThroughPosesGoalHandle::SharedPtr nav_through_poses_goal_handle_;
  TargetTrackingGoalHandle::SharedPtr target_tracking_goal_handle_;

  // The client used to control the nav2 stack
  nav2_lifecycle_manager::LifecycleManagerClient client_nav_;
  nav2_lifecycle_manager::LifecycleManagerClient client_loc_;
  std::shared_ptr<RealSenseClient> client_realsense_{nullptr};

  // nav2_lifecycle_manager::LifecycleManagerClient client_data_;
  nav2_lifecycle_manager::LifecycleManagerClient client_mapping_;
  nav2_lifecycle_manager::LifecycleManagerClient client_mcr_uwb_;
  rclcpp::TimerBase::SharedPtr nav_timer_;
  rclcpp::TimerBase::SharedPtr loc_timer_;
  rclcpp::TimerBase::SharedPtr waypoint_follow_timer_;
  rclcpp::TimerBase::SharedPtr through_pose_timer_;
  int status_;
  ActionType action_type_;
  // TODO(PDF):
  std::shared_ptr<nav2_util::LifecycleServiceClient> client_vision_manager_;
  std::shared_ptr<nav2_util::LifecycleServiceClient> client_tracking_manager_;
  rclcpp::Client<protocol::srv::AlgoManager>::SharedPtr client_vision_algo_;
  rclcpp::Service<BodyRegionT>::SharedPtr service_tracking_object_;
  rclcpp::Client<BodyRegionT>::SharedPtr client_tracking_object_;
  int32_t vision_action_client_feedback_;
  bool start_vision_tracking_;
  
  void GetNavStatus(int & status, ActionType & action_type);
  void HandleRelocCallback(const std_msgs::msg::Int32::SharedPtr msg)
  {
    reloc_status_ = static_cast<RelocStatus>(msg->data);
    INFO("%d", reloc_status_);

    if (reloc_topic_waiting_) {
      INFO("notify");
      reloc_cv_.notify_one();
      reloc_topic_waiting_ = false;
    }
  }

  void ResetReloc()
  {
    auto request = std::make_shared<std_srvs::srv::SetBool_Request>();
    request->data = true;
    if (!ServiceImpl(stop_loc_client_, request)) {
      ERROR("Failed to cancel Reloc because stopping service failed");
    }
    if (client_loc_.is_active() == nav2_lifecycle_manager::SystemStatus::ACTIVE) {
      if (!client_loc_.pause()) {
        ERROR("Failed to cancel Reloc because pause failed");
      }
    }
    reloc_status_ = RelocStatus::kIdle;
  }

  rclcpp_action::Server<Navigation>::SharedPtr navigation_server_;

  rclcpp_action::GoalResponse HandleNavigationGoal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const Navigation::Goal> goal);

  rclcpp_action::CancelResponse HandleNavigationCancel(
    const std::shared_ptr<GoalHandleNavigation> goal_handle);

  void HandleNavigationAccepted(
    const std::shared_ptr<GoalHandleNavigation> goal_handle);

  void FollowExecute(const std::shared_ptr<GoalHandleNavigation> goal_handle);

  /**
   * @brief Cancel robot's navigation
   *
   * @return true
   * @return false
   */
  bool CancelNavigation();

  /**
   * @brief Whether to report the dog's position in real time
   *
   * @param start If true report, false not report
   * @return true Report pose
   * @return false Not report pose
   */
  bool ReportRealtimeRobotPose(bool start);

  /**
   * @brief Stop build mapping
   * @return true
   * @return false
   */
  bool StopMapping();

  /**
   * @brief Manager all task's status
   */
  void TaskManager();

  // save goal handle to local
  std::shared_ptr<GoalHandleNavigation> goal_handle_;
  void SenResult();

  rclcpp::Publisher<protocol::msg::FollowPoints>::SharedPtr points_pub_;
  rclcpp::Subscription<protocol::msg::FollowPoints>::SharedPtr
    points_subscriber_;
  bool ServiceImpl(
    const rclcpp::Client<TriggerT>::SharedPtr,
    const std_srvs::srv::SetBool_Request::SharedPtr);
  void FollwPointCallback(const protocol::msg::FollowPoints::SharedPtr msg);
  std_msgs::msg::Header ReturnHeader();

  void set_running_navigation(bool state);
  bool running_navigation();

  bool running_navigation_ {false};
  rclcpp::Client<TriggerT>::SharedPtr start_mapping_client_;
  // rclcpp::Client<TriggerT>::SharedPtr stop_mapping_client_;
  rclcpp::Client<visualization::srv::Stop>::SharedPtr stop_mapping_client_;
  rclcpp::Client<TriggerT>::SharedPtr start_loc_client_;
  rclcpp::Client<TriggerT>::SharedPtr stop_loc_client_;

  // robot's realtime pose client
  rclcpp::Client<TriggerT>::SharedPtr realtime_pose_client_;

  rclcpp::CallbackGroup::SharedPtr callback_group_;
  RelocStatus reloc_status_{RelocStatus::kIdle};
  std::mutex reloc_mutex_;
  std::condition_variable reloc_cv_;
  bool reloc_topic_waiting_{false};

  std::mutex navigation_mutex_;
  std::condition_variable navigation_cond_;
  bool navigation_finished_ {false};
  std::shared_ptr<std::thread> task_managers_ {nullptr};

  bool send_result_flag_ = false;

  void set_send_result_flag(bool result);
};
}  // namespace carpo_navigation
#endif  // MOTION_CORE__MOTION_CORE_NODE_HPP_
