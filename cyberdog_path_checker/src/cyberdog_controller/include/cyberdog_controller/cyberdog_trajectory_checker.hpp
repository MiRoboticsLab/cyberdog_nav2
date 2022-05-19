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

#ifndef cyberdog_controller__cyberdog_controller_HPP_
#define cyberdog_controller__cyberdog_controller_HPP_

#include <memory>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "dwb_core/dwb_local_planner.hpp"
#include "nav2_core/controller.hpp"
#include "nav2_core/goal_checker.hpp"
#include "nav2_core/progress_checker.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_msgs/action/follow_path.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_msgs/msg/speed_limit.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav2_util/simple_action_server.hpp"
#include "nav_2d_utils/odom_subscriber.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2_ros/transform_listener.h"
using namespace std::chrono_literals;
using GoalStatus = action_msgs::msg::GoalStatus;
namespace cyberdog_controller {
/**
 * @class cyberdog_controller::TrajectoryChecker
 * @brief This class hosts variety of plugins of different algorithms to
 * complete control tasks from the exposed FollowPath action server.
 */
class TrajectoryChecker : public nav2_util::LifecycleNode {
 public:
  using ControllerMap =
      std::unordered_map<std::string, nav2_core::Controller::Ptr>;
  using GoalCheckerMap =
      std::unordered_map<std::string, nav2_core::GoalChecker::Ptr>;
  using NavigationGoalHandle =
      rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>;
  /**
   * @brief Constructor for cyberdog_controller::TrajectoryChecker
   */
  TrajectoryChecker();
  /**
   * @brief Destructor for cyberdog_controller::TrajectoryChecker
   */
  ~TrajectoryChecker();

 protected:
  /**
   * @brief Configures controller parameters and member variables
   *
   * Configures controller plugin and costmap; Initialize odom subscriber,
   * velocity publisher and follow path action server.
   * @param state LifeCycle Node's state
   * @return Success or Failure
   * @throw pluginlib::PluginlibException When failed to initialize controller
   * plugin
   */
  nav2_util::CallbackReturn on_configure(
      const rclcpp_lifecycle::State& state) override;
  /**
   * @brief Activates member variables
   *
   * Activates controller, costmap, velocity publisher and follow path action
   * server
   * @param state LifeCycle Node's state
   * @return Success or Failure
   */
  nav2_util::CallbackReturn on_activate(
      const rclcpp_lifecycle::State& state) override;
  /**
   * @brief Deactivates member variables
   *
   * Deactivates follow path action server, controller, costmap and velocity
   * publisher. Before calling deactivate state, velocity is being set to zero.
   * @param state LifeCycle Node's state
   * @return Success or Failure
   */
  nav2_util::CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State& state) override;
  /**
   * @brief Calls clean up states and resets member variables.
   *
   * Controller and costmap clean up state is called, and resets rest of the
   * variables
   * @param state LifeCycle Node's state
   * @return Success or Failure
   */
  nav2_util::CallbackReturn on_cleanup(
      const rclcpp_lifecycle::State& state) override;
  /**
   * @brief Called when in Shutdown state
   * @param state LifeCycle Node's state
   * @return Success or Failure
   */
  nav2_util::CallbackReturn on_shutdown(
      const rclcpp_lifecycle::State& state) override;

  using Action = nav2_msgs::action::FollowPath;
  using ActionServer = nav2_util::SimpleActionServer<Action>;

  // Our action server implements the FollowPath action
  std::unique_ptr<ActionServer> action_server_;

  void computeAndPublishVelocity();
  /**
   * @brief Calls setPlannerPath method with an updated path received from
   * action server
   */
  void updateGlobalPath();
  /**
   * @brief Calls velocity publisher to publish the velocity on "cmd_vel" topic
   * @param velocity Twist velocity to be published
   */
  void publishVelocity(const geometry_msgs::msg::TwistStamped& velocity);
  /**
   * @brief Calls velocity publisher to publish zero velocity
   */
  void publishZeroVelocity();

  // The controller needs a costmap node
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  std::unique_ptr<nav2_util::NodeThread> costmap_thread_;

  // Publishers and subscribers;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr
      vel_publisher_;
  nav2_core::Controller::Ptr controller_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;

  void cmd_vell_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
  geometry_msgs::msg::Twist cmd_vel_;
  std::mutex mut;
  std::shared_ptr<std::thread> planning_thread_;
  void controlLoop();
  double theta_range_;
  int num_th_samples_, num_x_samples_;
  double collision_trans_speed_, collision_rot_speed_;
  double controller_frequency_;
  double min_x_velocity_threshold_;
  double min_y_velocity_threshold_;
  double min_theta_velocity_threshold_;
  bool checkTrajectory(double vx_samp, double vy_samp, double vtheta_samp,
                       bool update_map);
  bool getRobotPose(geometry_msgs::msg::PoseStamped& pose);
  void setPlannerPath(const nav_msgs::msg::Path& path);
  nav_2d_msgs::msg::Twist2D getThresholdedTwist(
      const nav_2d_msgs::msg::Twist2D& twist);
  std::unique_ptr<nav_2d_utils::OdomSubscriber> odom_sub_;

  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr
      navigation_action_client_;
  rclcpp::Node::SharedPtr client_node_;
  bool startNavigation(geometry_msgs::msg::PoseStamped pose);
  NavigationGoalHandle::SharedPtr navigation_goal_handle_;
  std::chrono::milliseconds server_timeout_;
  bool calcualteGoal(double vx, double vy,
                     geometry_msgs::msg::PoseStamped& pose);
  bool cancleGoal();
  bool isGoalSent();
  bool poseValid(const geometry_msgs::msg::PoseStamped& pose);
  bool isValidCost(const unsigned char cost);
};

}  // namespace cyberdog_controller

#endif  // cyberdog_controller__cyberdog_controller_HPP_
