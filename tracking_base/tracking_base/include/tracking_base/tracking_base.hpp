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

#ifndef TRACKING_BASE__TRACKING_BASE_HPP_
#define TRACKING_BASE__TRACKING_BASE_HPP_

#include <memory>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

// #include "teb_local_planner/obstacles.h"
#include "teb_local_planner/teb_local_planner_ros.h"

#include "nav2_core/controller.hpp"
#include "nav2_core/progress_checker.hpp"
#include "nav2_core/goal_checker.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "tf2_ros/transform_listener.h"
#include "mcr_msgs/action/target_tracking.hpp"
#include "nav2_msgs/msg/speed_limit.hpp"
#include "nav_2d_utils/odom_subscriber.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/simple_action_server.hpp"
#include "nav2_util/robot_utils.hpp"
#include "pluginlib/class_loader.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tracking_base/target_manager.hpp"

namespace tracking_base
{
using namespace teb_local_planner;
class ProgressChecker;
/**
 * @class tracking_base::TrackingBase
 * @brief This class hosts variety of plugins of different algorithms to
 * complete control tasks from the exposed FollowPath action server.
 */
class TrackingBase : public nav2_util::LifecycleNode
{
public:
  /**
   * @brief Constructor for tracking_base::TrackingBase
   */
  TrackingBase();
  /**
   * @brief Destructor for tracking_base::TrackingBase
   */
  ~TrackingBase();

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
  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Activates member variables
   *
   * Activates controller, costmap, velocity publisher and follow path action
   * server
   * @param state LifeCycle Node's state
   * @return Success or Failure
   */
  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Deactivates member variables
   *
   * Deactivates follow path action server, controller, costmap and velocity
   * publisher. Before calling deactivate state, velocity is being set to zero.
   * @param state LifeCycle Node's state
   * @return Success or Failure
   */
  nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Calls clean up states and resets member variables.
   *
   * Controller and costmap clean up state is called, and resets rest of the
   * variables
   * @param state LifeCycle Node's state
   * @return Success or Failure
   */
  nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Called when in Shutdown state
   * @param state LifeCycle Node's state
   * @return Success or Failure
   */
  nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

  using Action = mcr_msgs::action::TargetTracking;
  using ActionServer = nav2_util::SimpleActionServer<Action>;

  // Our action server implements the FollowPath action
  std::unique_ptr<ActionServer> action_server_;

  /**
   * @brief FollowPath action server callback. Handles action server updates and
   * spins server until goal is reached
   *
   * Provides global path to controller received from action client. Twist
   * velocities for the robot are calculated and published using controller at
   * the specified rate till the goal is reached.
   * @throw nav2_core::PlannerException
   */
  void computeControl();

  /**
   * @brief Calculates velocity and publishes to "cmd_vel" topic
   */
  void computeAndPublishVelocity();
  /**
   * @brief Calls setPlannerPath method with an updated path received from
   * action server
   */
  bool updateRobotPoseAndTarget();
  /**
   * @brief Calls velocity publisher to publish the velocity on "cmd_vel" topic
   * @param velocity Twist velocity to be published
   */
  void publishVelocity(const geometry_msgs::msg::TwistStamped & velocity);
  /**
   * @brief Calls velocity publisher to publish zero velocity
   */
  void publishZeroVelocity();
  /**
   * @brief Checks if goal is reached
   * @return true or false
   */
  bool isGoalReached(const teb_local_planner::PoseSE2& s, 
                    const teb_local_planner::PoseSE2& g);
  /**
   * @brief Obtain current pose of the robot
   * @param pose To store current pose of the robot
   * @return true if able to obtain current pose of the robot, else false
   */
  bool getRobotPose(geometry_msgs::msg::PoseStamped & pose);

  void spin();
  void updateObstacleContainerWithCostmapConverter();
  void updateObstacleContainerWithCostmap();
  /**
   * @brief get the thresholded velocity
   * @param velocity The current velocity from odometry
   * @param threshold The minimum velocity to return non-zero
   * @return double velocity value
   */
  double getThresholdedVelocity(double velocity, double threshold)
  {
    return (std::abs(velocity) > threshold) ? velocity : 0.0;
  }

  /**
   * @brief get the thresholded Twist
   * @param Twist The current Twist from odometry
   * @return Twist Twist after thresholds applied
   */
  nav_2d_msgs::msg::Twist2D getThresholdedTwist(const nav_2d_msgs::msg::Twist2D & twist)
  {
    nav_2d_msgs::msg::Twist2D twist_thresh;
    twist_thresh.x = getThresholdedVelocity(twist.x, min_x_velocity_threshold_);
    twist_thresh.y = getThresholdedVelocity(twist.y, min_y_velocity_threshold_);
    twist_thresh.theta = getThresholdedVelocity(twist.theta, min_theta_velocity_threshold_);
    return twist_thresh;
  }
  // The controller needs a costmap node
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  std::unique_ptr<nav2_util::NodeThread> costmap_thread_;

  // Publishers and subscribers
  std::unique_ptr<nav_2d_utils::OdomSubscriber> odom_sub_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr vel_publisher_;
  rclcpp::Subscription<nav2_msgs::msg::SpeedLimit>::SharedPtr speed_limit_sub_;

  double controller_frequency_;
  double min_x_velocity_threshold_;
  double min_y_velocity_threshold_;
  double min_theta_velocity_threshold_;

  double transform_tolerance_;
  double dist_tolerance_sq_;
  double rot_vel_;

  // Whether we've published the single controller warning yet
  geometry_msgs::msg::Pose end_pose_;

  // Last time the controller generated a valid command
  rclcpp::Time last_valid_cmd_time_;

  // Current path container
  nav_msgs::msg::Path current_path_;

private:
  /**
    * @brief Callback for speed limiting messages
    * @param msg Shared pointer to nav2_msgs::msg::SpeedLimit
    */
  void speedLimitCallback(const nav2_msgs::msg::SpeedLimit::SharedPtr msg);

  // Definition of member variables
  rclcpp_lifecycle::LifecycleNode::WeakPtr nh_;
  rclcpp::Logger logger_{rclcpp::get_logger("TEBLocalPlanner")};
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Node::SharedPtr intra_proc_node_;
  // external objects (store weak pointers)
  nav2_costmap_2d::Costmap2D* costmap_; //!< Pointer to the 2d costmap (obtained from the costmap ros wrapper)
  // Transform listener
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;  
  TebConfig::UniquePtr cfg_; //!< Config class that stores and manages all related parameters
    
  // internal objects (memory management owned)
  PlannerInterfacePtr planner_; //!< Instance of the underlying optimal planner class
  ObstContainer obstacles_; //!< Obstacle vector that should be considered during local trajectory optimization
  ViaPointContainer via_points_; //!< Container of via-points that should be considered during local trajectory optimization
  TebVisualizationPtr visualization_; //!< Instance of the visualization class (local/global plan, obstacles, ...)
  std::shared_ptr<dwb_critics::ObstacleFootprintCritic> costmap_model_;
  FailureDetector failure_detector_; //!< Detect if the robot got stucked
  
  std::vector<geometry_msgs::msg::PoseStamped> global_plan_; //!< Store the current global plan
  
  pluginlib::ClassLoader<costmap_converter::BaseCostmapToPolygons> costmap_converter_loader_; //!< Load costmap converter plugins at runtime
  std::shared_ptr<costmap_converter::BaseCostmapToPolygons> costmap_converter_; //!< Store the current costmap_converter  

  //std::shared_ptr< dynamic_reconfigure::Server<TebLocalPlannerReconfigureConfig> > dynamic_recfg_; //!< Dynamic reconfigure server to allow config modifications at runtime
  rclcpp::Subscription<costmap_converter_msgs::msg::ObstacleArrayMsg>::SharedPtr custom_obst_sub_; //!< Subscriber for custom obstacles received via a ObstacleMsg.
  std::mutex custom_obst_mutex_; //!< Mutex that locks the obstacle array (multi-threaded)
  costmap_converter_msgs::msg::ObstacleArrayMsg custom_obstacle_msg_; //!< Copy of the most recent obstacle message

  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr via_points_sub_; //!< Subscriber for custom via-points received via a Path msg.
  bool custom_via_points_active_; //!< Keep track whether valid via-points have been received from via_points_sub_
  std::mutex via_point_mutex_; //!< Mutex that locks the via_points container (multi-threaded)

  PoseSE2 robot_pose_; //!< Store current robot pose
  PoseSE2 robot_goal_; //!< Store current robot goal
  geometry_msgs::msg::Twist robot_vel_; //!< Store current robot translational and angular velocity (vx, vy, omega)
  int no_infeasible_plans_; //!< Store how many times in a row the planner failed to find a feasible plan.
  RotType last_preferred_rotdir_; //!< Store recent preferred turning direction
  geometry_msgs::msg::Twist last_cmd_; //!< Store the last control command generated in computeVelocityCommands()
  
  std::vector<geometry_msgs::msg::Point> footprint_spec_; //!< Store the footprint of the robot 
  double robot_inscribed_radius_; //!< The radius of the inscribed circle of the robot (collision possible)
  double robot_circumscribed_radius; //!< The radius of the circumscribed circle of the robot
  
  std::string global_frame_; //!< The frame in which the controller will run
  std::string robot_base_frame_; //!< Used as the base frame id of the robot
    
  rclcpp::Time last_valid_target_; //!< Store at which time stamp the last oscillation was detected
  // flags
  bool initialized_; //!< Keeps track about the correct initialization of this class
  std::string name_; //!< Name of plugin ID

  // Subscription for parameter change
  rclcpp::AsyncParametersClient::SharedPtr parameters_client_;
  rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr parameter_event_sub_;

  std::shared_ptr<TargetManager> target_manager_;


};

}  // namespace tracking_base

#endif  // TRACKING_BASE__TRACKING_BASE_HPP_
