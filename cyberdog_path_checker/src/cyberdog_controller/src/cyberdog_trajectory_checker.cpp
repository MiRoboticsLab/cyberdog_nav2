// Copyright (c) 2019 Intel Corporation
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

#include "cyberdog_controller/cyberdog_trajectory_checker.hpp"

#include <Eigen/Core>
#include <chrono>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "nav2_core/exceptions.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav_2d_utils/conversions.hpp"
#include "nav_2d_utils/tf_help.hpp"
#include "rclcpp/create_publisher.hpp"
#include "rclcpp/create_service.hpp"
using namespace std::chrono_literals;

#define param_float(a, b, c)         \
  this->declare_parameter((a), (c)); \
  (b) = get_parameter(a).as_double();

#define param_bool(a, b, c)          \
  this->declare_parameter((a), (c)); \
  (b) = get_parameter(a).as_bool();

#define param_string(a, b, c)        \
  this->declare_parameter((a), (c)); \
  (b) = get_parameter(a).as_string();

#define param_int(a, b, c)           \
  this->declare_parameter((a), (c)); \
  (b) = get_parameter(a).as_int();

namespace cyberdog_controller {

TrajectoryChecker::TrajectoryChecker()
    : LifecycleNode("cyberdog_controller_server", "", true),
      server_timeout_(20) {
  RCLCPP_INFO(get_logger(), "Creating controller server");

  param_float("controller_frequency", controller_frequency_, 1000.0);
  param_int("num_th_samples", num_th_samples_, 20);
  param_int("num_x_samples", num_x_samples_, 10);
  param_float("theta_range", theta_range_, 0.7);
  param_float("translational_collision_speed", collision_trans_speed_, 0.0);
  param_float("rotational_collision_speed", collision_rot_speed_, 0.0);
  declare_parameter("min_x_velocity_threshold", rclcpp::ParameterValue(0.0001));
  declare_parameter("min_y_velocity_threshold", rclcpp::ParameterValue(0.0001));
  declare_parameter("min_theta_velocity_threshold",
                    rclcpp::ParameterValue(0.0001));
  // The costmap node is used in the implementation of the controller
  costmap_ros_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
      "local_costmap", std::string{get_namespace()}, "local_costmap");

  // Launch a thread to run the costmap node
  costmap_thread_ = std::make_unique<nav2_util::NodeThread>(costmap_ros_);
  auto options = rclcpp::NodeOptions().arguments(
      {"--ros-args --remap __node:=navigation_dialog_action_client"});
  client_node_ = std::make_shared<rclcpp::Node>("_", options);
  navigation_action_client_ =
      rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
          client_node_, "navigate_to_pose");
}

bool TrajectoryChecker::poseValid(const geometry_msgs::msg::PoseStamped &pose) {
  unsigned int cell_x, cell_y;
  if (!costmap_ros_->getCostmap()->worldToMap(
          pose.pose.position.x, pose.pose.position.y, cell_x, cell_y)) {
    RCLCPP_ERROR(get_logger(), "Trajectory Goes Off Grid.");
    return false;
  }

  unsigned char cost = costmap_ros_->getCostmap()->getCost(cell_x, cell_y);
  if (!isValidCost(cost)) {
    RCLCPP_ERROR(get_logger(), "Trajectory Hits Obstacle.");
    return false;
  }
  return true;
}

bool TrajectoryChecker::isValidCost(const unsigned char cost) {
  return cost != nav2_costmap_2d::LETHAL_OBSTACLE &&
         cost != nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE &&
         cost != nav2_costmap_2d::NO_INFORMATION;
}
bool TrajectoryChecker::calcualteEdgeStart(
    double vx, double vy, geometry_msgs::msg::PoseStamped &pose) {
  // get current pose.
  (void)vx;
  (void)vy;
  if (getRobotPose(pose)) {
    geometry_msgs::msg::PoseStamped tmp_pose;
    unsigned int size_x = costmap_ros_->getCostmap()->getSizeInCellsX(),
                 size_y = costmap_ros_->getCostmap()->getSizeInCellsY();

    // get index of the array.
    // int index = master_grid.getIndex(mx, my);
    for (unsigned int i = 0; i < size_x; i++) {
      for (unsigned int j = 0; j < size_y; j++) {
        // int index = master_grid.getIndex(i, j);
        // the index is the costmap buffer index;
        if (!isValidCost(costmap_ros_->getCostmap()->getCost(i, j))) {
          // the cost is invalid
          // set the nebigerhould field
          pose.pose.position.x = i;
          pose.pose.position.y = j;
          return true;
        }
      }
    }
    // while (i++) {
    //   tmp_pose.pose.position.x = pose.pose.position.x + i * vx;
    //   tmp_pose.pose.position.y = pose.pose.position.y + i * vy;
    //   unsigned int cell_x, cell_y;

    //   // out of bound of costmap
    //   if (!costmap_ros_->getCostmap()->worldToMap(
    //           pose.pose.position.x, pose.pose.position.y, cell_x, cell_y)) {
    //     RCLCPP_ERROR(get_logger(), "Goes Off Grid.");
    //     return false;
    //   }

    //   // is a obstacle edge point
    //   unsigned char cost = costmap_ros_->getCostmap()->getCost(cell_x,
    //   cell_y); if (!isValidCost(cost)) {
    //     RCLCPP_ERROR(get_logger(), "Found a Obstacle point.");
    //     return true;
    //   }
    // }
  }
  return false;
}
bool TrajectoryChecker::calcualteGoal(double vx, double vy,
                                      geometry_msgs::msg::PoseStamped &pose) {
  // get current pose.
  if (getRobotPose(pose)) {
    geometry_msgs::msg::PoseStamped tmp_pose;
    // tmp_pose.pose.position.x = pose.pose.position.x + 10 * vx;
    // tmp_pose.pose.position.y = pose.pose.position.y + 10 * vy;
    // pose = tmp_pose;
    // return true;
    for (int i = 100; i > 0; i /= 2) {
      tmp_pose.pose.position.x = pose.pose.position.x + i * vx;
      tmp_pose.pose.position.y = pose.pose.position.y + i * vy;
      if (poseValid(tmp_pose)) {
        pose = tmp_pose;
        return true;
      }
    }
  }

  return false;
}
bool TrajectoryChecker::isGoalSent() {
  if (!navigation_goal_handle_) {
    RCLCPP_ERROR(get_logger(), "no goal processing ");
    return false;
  }
  auto status = navigation_goal_handle_->get_status();
  if (status == GoalStatus::STATUS_ACCEPTED ||
      status == GoalStatus::STATUS_EXECUTING) {
    RCLCPP_ERROR(get_logger(), "goal is sent ");
    return true;
  }
  return false;
}
bool TrajectoryChecker::cancleGoal() {
  if (!navigation_goal_handle_) {
    RCLCPP_INFO(get_logger(), "nothing to cancel");
    return false;
  }
  if (navigation_goal_handle_) {
    auto future_cancel =
        navigation_action_client_->async_cancel_goal(navigation_goal_handle_);

    if (rclcpp::spin_until_future_complete(client_node_, future_cancel,
                                           server_timeout_) !=
        rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR(get_logger(), "Failed to cancel goal");
      return false;
    } else {
      navigation_goal_handle_.reset();
      RCLCPP_ERROR(get_logger(), "canceled navigation goal");
      return true;
    }
  }
  return true;
}

bool TrajectoryChecker::startNavigation(geometry_msgs::msg::PoseStamped pose) {
  auto is_action_server_ready =
      navigation_action_client_->wait_for_action_server(
          std::chrono::seconds(5));
  if (!is_action_server_ready) {
    RCLCPP_ERROR(get_logger(),
                 "navigate_to_pose action server is not available."
                 " Is the initial pose set?");
    return false;
  }

  // Send the goal pose
  nav2_msgs::action::NavigateToPose::Goal navigation_goal_;
  navigation_goal_.pose = pose;

  RCLCPP_INFO(get_logger(),
              "NavigateToPose will be called using the BT Navigator's default "
              "behavior tree.");

  // Enable result awareness by providing an empty lambda function
  auto send_goal_options = rclcpp_action::Client<
      nav2_msgs::action::NavigateToPose>::SendGoalOptions();

  send_goal_options.result_callback = [this](auto) {
    RCLCPP_ERROR(get_logger(), "Get navigate to poses result");
    navigation_goal_handle_.reset();
  };

  auto future_goal_handle = navigation_action_client_->async_send_goal(
      navigation_goal_, send_goal_options);

  if (rclcpp::spin_until_future_complete(client_node_, future_goal_handle,
                                         server_timeout_) !=
      rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(get_logger(), "Send goal call failed");
    return false;
  }

  // Get the goal handle and save so that we can check on completion in the
  // timer callback
  navigation_goal_handle_ = future_goal_handle.get();
  if (!navigation_goal_handle_) {
    RCLCPP_ERROR(get_logger(), "Goal was rejected by server");
    return false;
  }
  return true;
}
#define EDGE_MOVE 1
TrajectoryChecker::~TrajectoryChecker() { costmap_thread_.reset(); }

void TrajectoryChecker::controlLoop() {
  rclcpp::Rate r(controller_frequency_);
  const POINT linkcode[8] = {{1, 0},  {1, 1},   {0, 1},  {-1, 1},
                             {-1, 0}, {-1, -1}, {0, -1}, {1, -1}};
  while (rclcpp::ok()) {
    geometry_msgs::msg::PoseStamped goal_pose;
    if (calcualteEdgeStart(1, 2, goal_pose)) {
      POINT start;
      start.x = goal_pose.pose.position.x;
      start.y = goal_pose.pose.position.y;
      RCLCPP_INFO(get_logger(), "Start at (%d:%d)", start.x, start.y);
      ivChainCode chaincode;
      TracingContour(&start, &chaincode);
      nav_msgs::msg::Path path;
      path.header.frame_id = "map";  // costmap_ros_->getBaseFrameID();
      path.header.stamp = now();
      RCLCPP_INFO(get_logger(), "iters:");
      geometry_msgs::msg::PoseStamped pose;
      POINT tmp_pose;

      tmp_pose.x = start.x;
      tmp_pose.y = start.y;

      double world_x, world_y;

      for (auto iter : chaincode) {
        tmp_pose.x = tmp_pose.x + linkcode[iter].x;
        tmp_pose.y = tmp_pose.y + linkcode[iter].y;
        costmap_ros_->getCostmap()->mapToWorld(tmp_pose.x, tmp_pose.y, world_x,
                                               world_y);
        RCLCPP_INFO(get_logger(), "%d ", iter);
        pose.pose.position.x = world_x;
        pose.pose.position.y = world_y;
        path.poses.push_back(pose);
      }
      for (auto pose_iter : path.poses) {
        RCLCPP_INFO(get_logger(), "%f, %f ", pose_iter.pose.position.x,
                    pose_iter.pose.position.y);
      }
      path_pub_->publish(path);
    } else {
      RCLCPP_INFO(get_logger(), "can not found a start point ");
    }

#if 0
    RCLCPP_INFO(get_logger(), "controlLoop looping");
    Eigen::Vector3f desired_vel = Eigen::Vector3f::Zero();
    // we'll copy over odometry and velocity data for planning
    {
      std::lock_guard<std::mutex> lk(mut);
      desired_vel[0] = cmd_vel_.linear.x;
      desired_vel[1] = cmd_vel_.linear.y;
      desired_vel[2] = cmd_vel_.angular.z;
    }

    // first, we'll check the trajectory that the user sent in... if its
    // legal... we'll just follow it
    if (checkTrajectory(desired_vel[0], desired_vel[1], desired_vel[2], true)) {
#ifdef USE_NAV_FOR_AVOIDANCE
      if (isGoalSent()) {
        cancleGoal();
      }
#endif
      geometry_msgs::msg::Twist cmd;
      cmd.linear.x = desired_vel[0];
      cmd.linear.y = desired_vel[1];
      cmd.angular.z = desired_vel[2];
      vel_publisher_->publish(cmd);

      r.sleep();
      continue;
    }

    double dth = (theta_range_) / double(num_th_samples_);
    double dx = desired_vel[0] / double(num_x_samples_);
    double start_th = desired_vel[2] - theta_range_ / 2.0;

    Eigen::Vector3f best = Eigen::Vector3f::Zero();
    double best_dist = DBL_MAX;
    bool trajectory_found = false;

    // if we don't have a valid trajectory... we'll start checking others in
    // the angular range specified
    for (int i = 0; i < /*num_x_samples_*/ 1; ++i) {
      Eigen::Vector3f check_vel = Eigen::Vector3f::Zero();
      check_vel[0] = desired_vel[0] - i * dx;
      check_vel[1] = desired_vel[1];
      check_vel[2] = start_th;
      for (int j = 0; j < num_th_samples_; ++j) {
        check_vel[2] = start_th + j * dth;
        if (checkTrajectory(check_vel[0], check_vel[1], check_vel[2], false)) {
          // if we have a legal trajectory, we'll score it based on its
          // distance to our desired velocity
          Eigen::Vector3f diffs = (desired_vel - check_vel);
          double sq_dist =
              diffs[0] * diffs[0] + diffs[1] * diffs[1] + diffs[2] * diffs[2];

          // if we have a trajectory that is better than our best one so
          // far, we'll take it
          if (sq_dist < best_dist) {
            best = check_vel;
            best_dist = sq_dist;
            trajectory_found = true;
            break;
          }
        }
      }
      if (trajectory_found) {
        break;
      }
    }
    RCLCPP_ERROR(get_logger(), "trajectory_found : ------- %d",
                 trajectory_found);
    // check if best is still zero, if it is... scale the original
    // trajectory based on the collision_speed requested but we only need to
    // do this if the user has set a non-zero collision speed
    if (!trajectory_found &&
        (collision_trans_speed_ > 0.0 || collision_rot_speed_ > 0.0)) {
      double trans_scaling_factor = 0.0;
      double rot_scaling_factor = 0.0;
      double scaling_factor = 0.0;

      if (fabs(desired_vel[0]) > 0 && fabs(desired_vel[1]) > 0)
        trans_scaling_factor =
            std::min(collision_trans_speed_ / fabs(desired_vel[0]),
                     collision_trans_speed_ / fabs(desired_vel[1]));
      else if (fabs(desired_vel[0]) > 0)
        trans_scaling_factor = collision_trans_speed_ / (fabs(desired_vel[0]));
      else if (fabs(desired_vel[1]) > 0)
        trans_scaling_factor = collision_trans_speed_ / (fabs(desired_vel[1]));

      if (fabs(desired_vel[2]) > 0)
        rot_scaling_factor = collision_rot_speed_ / (fabs(desired_vel[2]));

      if (collision_trans_speed_ > 0.0 && collision_rot_speed_ > 0.0)
        scaling_factor = std::min(trans_scaling_factor, rot_scaling_factor);
      else if (collision_trans_speed_ > 0.0)
        scaling_factor = trans_scaling_factor;
      else if (collision_rot_speed_ > 0.0)
        scaling_factor = rot_scaling_factor;

      // apply the scaling factor
      best = scaling_factor * best;
    }
#ifdef USE_NAV_FOR_AVOIDANCE
    RCLCPP_INFO(get_logger(), "vx, vy: pose:(%f,%f)", desired_vel[0],
                desired_vel[1]);

    if (!trajectory_found && ((desired_vel[0] != 0) || (desired_vel[1] != 0))) {
      if (!isGoalSent()) {
        geometry_msgs::msg::PoseStamped goal_pose;
        if (calcualteGoal(desired_vel[0], desired_vel[1], goal_pose)) {
          goal_pose.header.frame_id = costmap_ros_->getBaseFrameID();
          goal_pose.header.stamp = now();
          RCLCPP_ERROR(get_logger(),
                       "switch navigation mode, target pose:(%f,%f)",
                       goal_pose.pose.position.x, goal_pose.pose.position.y);
          startNavigation(goal_pose);
        }
      }
    } else {
      cancleGoal();
      geometry_msgs::msg::Twist best_cmd;
      best_cmd.linear.x = best[0];
      best_cmd.linear.y = best[1];
      best_cmd.angular.z = best[2];
      vel_publisher_->publish(best_cmd);
    }
#elif defined EDGE_MOVE
    const POINT linkcode[8] = {{1, 0},  {1, 1},   {0, 1},  {-1, 1},
                               {-1, 0}, {-1, -1}, {0, -1}, {1, -1}};
    geometry_msgs::msg::PoseStamped goal_pose;
    if (calcualteEdgeStart(desired_vel[0], desired_vel[1], goal_pose)) {
      POINT start;
      start.x = goal_pose.pose.position.x;
      start.y = goal_pose.pose.position.y;
      RCLCPP_INFO(get_logger(), "Start at (%d:%d)", start.x, start.y);
      ivChainCode chaincode;
      TracingContour(&start, &chaincode);
      nav_msgs::msg::Path path;
      path.header.frame_id = costmap_ros_->getBaseFrameID();
      path.header.stamp = now();
      RCLCPP_INFO(get_logger(), "iters:");
      for (auto iter : chaincode) {
        RCLCPP_INFO(get_logger(), "%d ", iter);
        geometry_msgs::msg::PoseStamped pose;
        pose.pose.position.x = goal_pose.pose.position.x + linkcode[iter].x;
        pose.pose.position.y = goal_pose.pose.position.y + linkcode[iter].y;
        path.poses.push_back(pose);
      }
      if (chaincode.size() > 5) path_pub_->publish(path);
    } else {
      RCLCPP_INFO(get_logger(), "can not found a start point ");
    }

#else
    geometry_msgs::msg::Twist best_cmd;
    best_cmd.linear.x = best[0];
    best_cmd.linear.y = best[1];
    best_cmd.angular.z = best[2];
    vel_publisher_->publish(best_cmd);
#endif
#endif
    r.sleep();
  }
}

nav2_util::CallbackReturn TrajectoryChecker::on_configure(
    const rclcpp_lifecycle::State &state) {
  auto node = shared_from_this();
  get_parameter("min_x_velocity_threshold", min_x_velocity_threshold_);
  get_parameter("min_y_velocity_threshold", min_y_velocity_threshold_);
  get_parameter("min_theta_velocity_threshold", min_theta_velocity_threshold_);
  RCLCPP_INFO(get_logger(), "Configuring cyberdog controller interface");
  costmap_ros_->on_configure(state);

  vel_publisher_ = create_publisher<geometry_msgs::msg::Twist>(
      "cmd_vel_filter", rclcpp::SystemDefaultsQoS());

  cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", rclcpp::SystemDefaultsQoS(),
      std::bind(&TrajectoryChecker::cmd_vell_callback, this,
                std::placeholders::_1));

  path_pub_ = create_publisher<nav_msgs::msg::Path>(
      "Path", rclcpp::SystemDefaultsQoS());

  controller_ = std::make_unique<dwb_core::DWBLocalPlanner>();
  controller_->configure(node, "FollowPath", costmap_ros_->getTfBuffer(),
                         costmap_ros_);
  odom_sub_ = std::make_unique<nav_2d_utils::OdomSubscriber>(node);
  return nav2_util::CallbackReturn::SUCCESS;
}

void TrajectoryChecker::cmd_vell_callback(
    const geometry_msgs::msg::Twist::SharedPtr msg) {
  std::lock_guard<std::mutex> lk(mut);
  cmd_vel_ = *msg;
}
nav2_util::CallbackReturn TrajectoryChecker::on_activate(
    const rclcpp_lifecycle::State &state) {
  RCLCPP_INFO(get_logger(), "Cyberdog controller Activating");
  costmap_ros_->on_activate(state);
  vel_publisher_->on_activate();
  path_pub_->on_activate();
  controller_->activate();
  planning_thread_ =
      std::make_shared<std::thread>(&TrajectoryChecker::controlLoop, this);
  // create bond connection
  createBond();
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn TrajectoryChecker::on_deactivate(
    const rclcpp_lifecycle::State &state) {
  RCLCPP_INFO(get_logger(), "Deactivating");
  auto node = shared_from_this();
  ControllerMap::iterator it;
  costmap_ros_->on_deactivate(state);
  vel_publisher_->on_deactivate();
  controller_->deactivate();
  // destroy bond connection
  destroyBond();
  path_pub_->on_deactivate();
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn TrajectoryChecker::on_cleanup(
    const rclcpp_lifecycle::State &state) {
  RCLCPP_INFO(get_logger(), "Cleaning up");
  costmap_ros_->on_cleanup(state);

  vel_publisher_.reset();
  path_pub_.reset();
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn TrajectoryChecker::on_shutdown(
    const rclcpp_lifecycle::State &) {
  RCLCPP_INFO(get_logger(), "Shutting down");
  return nav2_util::CallbackReturn::SUCCESS;
}
bool TrajectoryChecker::getRobotPose(geometry_msgs::msg::PoseStamped &pose) {
  geometry_msgs::msg::PoseStamped current_pose;
  if (!costmap_ros_->getRobotPose(current_pose)) {
    return false;
  }
  pose = current_pose;
  return true;
}

void TrajectoryChecker::setPlannerPath(const nav_msgs::msg::Path &path) {
  if (path.poses.empty()) {
    throw nav2_core::PlannerException("Invalid path, Path is empty.");
  }
  controller_->setPlan(path);
}

double getThresholdedVelocity(double velocity, double threshold) {
  return (std::abs(velocity) > threshold) ? velocity : 0.0;
}

nav_2d_msgs::msg::Twist2D TrajectoryChecker::getThresholdedTwist(
    const nav_2d_msgs::msg::Twist2D &twist) {
  nav_2d_msgs::msg::Twist2D twist_thresh;
  twist_thresh.x = getThresholdedVelocity(twist.x, min_x_velocity_threshold_);
  twist_thresh.y = getThresholdedVelocity(twist.y, min_y_velocity_threshold_);
  twist_thresh.theta =
      getThresholdedVelocity(twist.theta, min_theta_velocity_threshold_);
  return twist_thresh;
}

bool TrajectoryChecker::checkTrajectory(double vx_samp, double vy_samp,
                                        double vtheta_samp, bool update_map) {
  geometry_msgs::msg::PoseStamped pose;
  if (getRobotPose(pose)) {
    if (update_map) {
      // we also want to clear the robot footprint from the costmap we're
      // using costmap_ros_->clearRobotFootprint();

      // make sure to update the costmap we'll use for this cycle
      // costmap_ros_->getCostmapCopy(costmap_);

      // we need to give the planne some sort of global plan, since we're only
      // checking for legality we'll just give the robots current position
      nav_msgs::msg::Path path;
      path.header.frame_id = costmap_ros_->getBaseFrameID();
      path.header.stamp = now();
      path.poses.push_back(pose);
      setPlannerPath(path);
    }
    nav_2d_msgs::msg::Twist2D twist =
        getThresholdedTwist(odom_sub_->getTwist());
    return controller_->checkTrajectory(pose, twist, vx_samp, vy_samp,
                                        vtheta_samp);
  }
  RCLCPP_WARN(get_logger(),
              "Failed to get the pose of the robot. No trajectories will pass "
              "as legal in this case.");
  return false;
}

// 轮廓跟踪
// 1. pImageData   图像数据
// 2. nWidth       图像宽度     // master_grid.getSizeInCellsX()
// 3. nHeight      图像高度     // master_grid.getSizeInCellsY()
// 4. nWidthStep   图像行大小   // master_grid.getSizeInCellsX()
// 5. pStart       起始点      // given by tracjotry checker.
// 6. pChainCode   链码表      // result
bool TrajectoryChecker::TracingContour(POINT *pStart, ivChainCode *pChainCode) {
  int nWidth = costmap_ros_->getCostmap()->getSizeInCellsX();
  int nHeight = costmap_ros_->getCostmap()->getSizeInCellsY();
  int i = 0;
  int j = 0;
  int k = 0;
  int x = 0;
  int y = 0;
  bool bTracing = false;
  POINT ptCurrent;
  POINT ptTemp = {0, 0};
  ptCurrent.x = pStart->x;
  ptCurrent.y = pStart->y;

  const POINT ptOffset[8] = {{1, 0},  {1, 1},   {0, 1},  {-1, 1},
                             {-1, 0}, {-1, -1}, {0, -1}, {1, -1}};
  // 清空起始点与链码表
  // pStart->x = 0;
  // pStart->y = 0;
  pChainCode->clear();
  // // 轮廓起点
  // for (y = 0; y < nHeight; y++) {
  //   pLine = pImageData + nWidthStep * y;
  //   for (x = 0; x < nWidth; x++) {
  //     if (pLine[x] == 0xFF) {
  //       bTracing = true;
  //       pStart->x = x;
  //       pStart->y = y;
  //       ptCurrent.x = x;
  //       ptCurrent.y = y;
  //     }
  //   }
  // }
  // 轮廓跟踪
  bTracing = true;
  while (bTracing) {
    bTracing = false;
    for (i = 0; i < 8; i++, k++) {
      k &= 0x07;
      x = ptCurrent.x + ptOffset[k].x;
      y = ptCurrent.y + ptOffset[k].y;
      if (x >= 0 && x < nWidth && y >= 0 && y < nHeight) {
        // 判断是否为轮廓点
        if (!isValidCost(costmap_ros_->getCostmap()->getCost(x, y))) {
          // pImageData[nWidthStep * y + x] == 0xFF
          for (j = 0; j < 8; j += 2) {
            ptTemp.x = x + ptOffset[j].x;
            ptTemp.y = y + ptOffset[j].y;
            if (ptTemp.x >= 0 && ptTemp.x < nWidth && ptTemp.y >= 0 &&
                ptTemp.y < nHeight) {
              if (isValidCost(costmap_ros_->getCostmap()->getCost(ptTemp.x,
                                                                  ptTemp.y))) {
                //  pImageData[nWidthStep * ptTemp.y + ptTemp.x] == 0
                // isValidCost(costmap_ros_->getCostmap()->getCost(ptTemp.x,
                // ptTemp.y))
                bTracing = true;
                ptCurrent.x = x;
                ptCurrent.y = y;
                pChainCode->push_back(k);
                break;
              }
            }
          }
        }
      }
      if (bTracing) {
        // 如果当前点为轮廓起点
        if (pStart->x == ptCurrent.x && pStart->y == ptCurrent.y) {
          // 则跟踪完毕
          bTracing = false;
        }
        break;
      }
    }
    k += 0x06;
  }
  return true;
}

}  // namespace cyberdog_controller
