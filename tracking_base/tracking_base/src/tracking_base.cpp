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

#include <chrono>
#include <vector>
#include <memory>
#include <string>
#include <utility>
#include <limits>
#include <boost/algorithm/string.hpp>
#include "tf2_ros/create_timer_ros.h"
#include "nav2_core/exceptions.hpp"
#include "nav_2d_utils/conversions.hpp"
#include "nav_2d_utils/tf_help.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "tracking_base/tracking_base.hpp"

using namespace std::chrono_literals;

namespace tracking_base 
{

TrackingBase::TrackingBase()
: LifecycleNode("tracking_base", "", true), name_{"teb"},
  costmap_ros_(nullptr), tf_buffer_(nullptr), cfg_(new TebConfig()), costmap_model_(nullptr), 
  costmap_converter_loader_("costmap_converter", "costmap_converter::BaseCostmapToPolygons"),
  custom_via_points_active_(false), no_infeasible_plans_(0), last_preferred_rotdir_(RotType::none), initialized_(false)
{
  RCLCPP_INFO(get_logger(), "Creating tracking base...");

  declare_parameter("controller_frequency", 10.0);
  declare_parameter("global_frame", rclcpp::ParameterValue("odom"));
  declare_parameter("base_frame", rclcpp::ParameterValue("base_link"));
  declare_parameter("transform_tolerance", rclcpp::ParameterValue(0.3));
  declare_parameter("dist_tolerance", rclcpp::ParameterValue(0.5));
  declare_parameter("rot_vel", rclcpp::ParameterValue(0.5));

  declare_parameter("speed_limit_topic", rclcpp::ParameterValue("speed_limit"));
  
}

TrackingBase::~TrackingBase()
{
  costmap_thread_.reset();
}

nav2_util::CallbackReturn
TrackingBase::on_configure(const rclcpp_lifecycle::State & state)
{
  auto node = shared_from_this();
  target_manager_ = std::make_shared<TargetManager>(node);

  RCLCPP_INFO(get_logger(), "Configuring controller interface");

  get_parameter("controller_frequency", controller_frequency_);
  RCLCPP_INFO(get_logger(), "Controller frequency set to %.4fHz", controller_frequency_);

  std::string speed_limit_topic;
  get_parameter("speed_limit_topic", speed_limit_topic);
  get_parameter("global_frame", global_frame_);
  get_parameter("base_frame", robot_base_frame_);
  get_parameter("transform_tolerance", transform_tolerance_);
  get_parameter("dist_tolerance", dist_tolerance_sq_);
  get_parameter("rot_vel", rot_vel_);
  dist_tolerance_sq_ *= dist_tolerance_sq_;
  // Create the transform-related objects
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(rclcpp_node_->get_clock());
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    rclcpp_node_->get_node_base_interface(),
    rclcpp_node_->get_node_timers_interface());
  tf_buffer_->setCreateTimerInterface(timer_interface);
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // declare parameters (ros2-dashing)
  intra_proc_node_.reset( 
          new rclcpp::Node("costmap_converter", node->get_namespace(), 
            rclcpp::NodeOptions()));
  cfg_->declareParameters(node, name_);

  // get parameters of TebConfig via the nodehandle and override the default config
  cfg_->loadRosParamFromNodeHandle(node, name_);
    
  // reserve some memory for obstacles
  obstacles_.reserve(500);
        
  // create robot footprint/contour model for optimization
  RobotFootprintModelPtr robot_model = std::make_shared<PointRobotFootprint>();
    
  planner_ = PlannerInterfacePtr(new TebOptimalPlanner(node, *cfg_.get(), &obstacles_, robot_model, visualization_, &via_points_));
  RCLCPP_INFO(logger_, "Parallel planning in distinctive topologies disabled.");

  visualization_ = std::make_shared<TebVisualization>(node, *cfg_);
  visualization_->on_configure();
  planner_->setVisualization(visualization_); 
  clock_ = node->get_clock();

  // The costmap node is used in the implementation of the controller
  costmap_ros_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
    "costmap", std::string{get_namespace()}, "costmap");
  // Launch a thread to run the costmap node
  costmap_thread_ = std::make_unique<nav2_util::NodeThread>(costmap_ros_);
  costmap_ros_->on_configure(state);
  // init other variables
  costmap_ = costmap_ros_->getCostmap(); // locking should be done in MoveBase.
  costmap_model_ = std::make_shared<dwb_critics::ObstacleFootprintCritic>();
  std::string costmap_model_name("costmap_model");
  costmap_model_->initialize(node, costmap_model_name, name_, costmap_ros_);

  global_frame_ = costmap_ros_->getGlobalFrameID();
  cfg_->map_frame = global_frame_; // TODO
  robot_base_frame_ = costmap_ros_->getBaseFrameID();

  odom_sub_ = std::make_unique<nav_2d_utils::OdomSubscriber>(node);
  vel_publisher_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);

  //Initialize a costmap to polygon converter
  if (!cfg_->obstacles.costmap_converter_plugin.empty())
  {
    try
    {
      costmap_converter_ = costmap_converter_loader_.createSharedInstance(cfg_->obstacles.costmap_converter_plugin);
      std::string converter_name = costmap_converter_loader_.getName(cfg_->obstacles.costmap_converter_plugin);
      RCLCPP_INFO(logger_, "library path : %s", costmap_converter_loader_.getClassLibraryPath("costmap_converter").c_str());
      // replace '::' by '/' to convert the c++ namespace to a NodeHandle namespace
      boost::replace_all(converter_name, "::", "/");

      costmap_converter_->setOdomTopic(cfg_->odom_topic);
      costmap_converter_->initialize(intra_proc_node_);
      costmap_converter_->setCostmap2D(costmap_);
      const auto rate = std::make_shared<rclcpp::Rate>((double)cfg_->obstacles.costmap_converter_rate);
      costmap_converter_->startWorker(rate, costmap_, cfg_->obstacles.costmap_converter_spin_thread);
      RCLCPP_INFO(logger_, "Costmap conversion plugin %s loaded.", cfg_->obstacles.costmap_converter_plugin.c_str());
    }
    catch(pluginlib::PluginlibException& ex)
    {
      RCLCPP_INFO(logger_,
                 "The specified costmap converter plugin cannot be loaded. All occupied costmap cells are treaten as point obstacles. Error message: %s", ex.what());
      costmap_converter_.reset();
    }
  }
  else {
    RCLCPP_INFO(logger_, "No costmap conversion plugin specified. All occupied costmap cells are treaten as point obstacles.");
  }

  // Set subscribtion to the speed limiting topic
  speed_limit_sub_ = create_subscription<nav2_msgs::msg::SpeedLimit>(
    speed_limit_topic, rclcpp::QoS(10),
    std::bind(&TrackingBase::speedLimitCallback, this, std::placeholders::_1));

  // Create the action server that we implement with our followPath method
  action_server_ = std::make_unique<ActionServer>(
    rclcpp_node_, "tracking_target",
    std::bind(&TrackingBase::computeControl, this));

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
TrackingBase::on_activate(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Activating");

  costmap_ros_->on_activate(state);
  //activate controller
  vel_publisher_->on_activate();
  action_server_->activate();
  visualization_->on_activate();

  // create bond connection
  createBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
TrackingBase::on_deactivate(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Deactivating");

  action_server_->deactivate();
  visualization_->on_deactivate();
  //controller deactiveate
  //costmap deactivate
  costmap_ros_->on_deactivate(state);

  publishZeroVelocity();
  vel_publisher_->on_deactivate();

  // destroy bond connection
  destroyBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
TrackingBase::on_cleanup(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Cleaning up");

  // Cleanup the helper classes
  // costmap cleanup
  costmap_ros_->on_cleanup(state);

  // Release any allocated resources
  action_server_.reset();
  visualization_->on_cleanup();
  odom_sub_.reset();
  vel_publisher_.reset();
  speed_limit_sub_.reset();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
TrackingBase::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Shutting down");
  return nav2_util::CallbackReturn::SUCCESS;
}

void TrackingBase::spin()
{
  geometry_msgs::msg::PoseStamped p;
  geometry_msgs::msg::TwistStamped cmd_vel;
  
  double scale = 1.0;
  target_manager_->getRawTarget(p);
  if (p.pose.position.y < 0){
    scale = -1.0;
  }

  cmd_vel.header.stamp = clock_->now();
  cmd_vel.header.frame_id = robot_base_frame_;
  cmd_vel.twist.linear.x = 0;
  cmd_vel.twist.linear.y = 0;
  cmd_vel.twist.angular.z = rot_vel_ * scale;
  publishVelocity(cmd_vel);
}

void TrackingBase::computeControl()
{
  RCLCPP_INFO(get_logger(), "Received a goal, begin computing control effort.");

  try {
    double keep_distance = action_server_->get_current_goal()->keep_distance;
    // if (findControllerId(c_name, current_controller)) {
    //   current_controller_ = current_controller;
    // } else {
    //   action_server_->terminate_current();
    //   return;
    // }

    last_valid_cmd_time_ = now();
    std::shared_ptr<Action::Feedback> feedback = std::make_shared<Action::Feedback>();
    feedback->current_distance = 1.0;
    feedback->exception_code = nav2_core::NOEXCEPTION;
    rclcpp::WallRate loop_rate(controller_frequency_);
    while (rclcpp::ok()) {
      if (!loop_rate.sleep()) {
        RCLCPP_WARN(
          get_logger(), "Control loop missed its desired rate of %.4fHz",
          controller_frequency_);
      }

      if (action_server_ == nullptr || !action_server_->is_server_active()) {
        RCLCPP_DEBUG(get_logger(), "Action server unavailable or inactive. Stopping.");
        return;
      }

      if (action_server_->is_cancel_requested()) {
        RCLCPP_INFO(get_logger(), "Goal was canceled. Stopping the robot.");
        action_server_->terminate_all();
        publishZeroVelocity();
        return;
      }


      // Don't compute a trajectory until costmap is valid (after clear costmap)
      rclcpp::Rate r(100);
      while (!costmap_ros_->isCurrent()) {
        r.sleep();
      }


      action_server_->publish_feedback(feedback);

      if (!updateRobotPoseAndTarget()){
        feedback->exception_code = nav2_core::DETECTOREXCEPTION;
        spin();
        continue;
      }

      if (isGoalReached(robot_pose_, robot_goal_)){
        // RCLCPP_INFO(get_logger(), "Reached the goal!");
        continue;
      }

      teb_local_planner::PoseSE2 k = robot_pose_ - robot_goal_;
      feedback->current_distance = std::hypot(k.x(), k.y());
      feedback->exception_code = nav2_core::NOEXCEPTION;

      obstacles_.clear();
      updateObstacleContainerWithCostmapConverter();
      geometry_msgs::msg::Twist twist = nav_2d_utils::twist2Dto3D(odom_sub_->getTwist());
      planner_->plan(robot_pose_, robot_goal_, &twist); // hardcoded start and goal for testing purposes
      planner_->visualize();
      visualization_->publishObstacles(obstacles_);
      visualization_->publishViaPoints(via_points_);
      visualization_->publishGlobalPlan(global_plan_);
  
      computeAndPublishVelocity();

    }
  } catch (nav2_core::PlannerException & e) {
    RCLCPP_ERROR(this->get_logger(), e.what());
    publishZeroVelocity();
    action_server_->terminate_current();
    return;
  }

  RCLCPP_DEBUG(get_logger(), "Controller succeeded, setting result");

  publishZeroVelocity();

  // TODO(orduno) #861 Handle a pending preemption and set controller name
  action_server_->succeeded_current();
}

void TrackingBase::computeAndPublishVelocity()
{
  geometry_msgs::msg::TwistStamped cmd_vel;
  
  cmd_vel.header.stamp = clock_->now();
  cmd_vel.header.frame_id = robot_base_frame_;
  cmd_vel.twist.linear.x = 0;
  cmd_vel.twist.linear.y = 0;
  cmd_vel.twist.angular.z = 0;  
  // Get the velocity command for this sampling interval
  if (!planner_->getVelocityCommand(cmd_vel.twist.linear.x, cmd_vel.twist.linear.y, cmd_vel.twist.angular.z, cfg_->trajectory.control_look_ahead_poses))
  {
    planner_->clearPlanner();
    throw nav2_core::PlannerException(
      std::string("TebLocalPlannerROS: velocity command invalid. Resetting planner...")
    );
  }
  publishVelocity(cmd_vel);
}

bool TrackingBase::updateRobotPoseAndTarget()
{  
  geometry_msgs::msg::PoseStamped target;
  if(!target_manager_->getTarget(target)){
    RCLCPP_WARN(get_logger(), "Invalid target, attempting to retrieve...");
    return false;
  }
  geometry_msgs::msg::PoseStamped robot_pose;
  if (!getRobotPose(robot_pose)){
    RCLCPP_WARN(get_logger(), "Invalid robot pose, attempting to retrieve...");
    return false;
  }

  robot_pose_ = teb_local_planner::PoseSE2(robot_pose.pose);
  robot_goal_ = teb_local_planner::PoseSE2(target.pose);
  return true;
}

void TrackingBase::publishVelocity(const geometry_msgs::msg::TwistStamped & velocity)
{
  auto cmd_vel = std::make_unique<geometry_msgs::msg::Twist>(velocity.twist);
  if (vel_publisher_->is_activated() && vel_publisher_->get_subscription_count() > 0) {
    RCLCPP_INFO(get_logger(), "[TrackingBase] cmd_vel [vx = %lf, vy = %lf, w = %lf]",
      cmd_vel->linear.x, cmd_vel->linear.y, cmd_vel->angular.z );
    vel_publisher_->publish(std::move(cmd_vel));
  }
}

void TrackingBase::publishZeroVelocity()
{
  geometry_msgs::msg::TwistStamped velocity;
  velocity.twist.angular.x = 0;
  velocity.twist.angular.y = 0;
  velocity.twist.angular.z = 0;
  velocity.twist.linear.x = 0;
  velocity.twist.linear.y = 0;
  velocity.twist.linear.z = 0;
  velocity.header.frame_id = "base_link";
  velocity.header.stamp = now();
  publishVelocity(velocity);
}

bool TrackingBase::isGoalReached(const teb_local_planner::PoseSE2& s, 
                                 const teb_local_planner::PoseSE2& g)
{
  teb_local_planner::PoseSE2 k = s - g;
  return k.x() * k.x() + k.y() * k.y() < dist_tolerance_sq_;
}

bool TrackingBase::getRobotPose(geometry_msgs::msg::PoseStamped & pose)
{
  geometry_msgs::msg::PoseStamped current_pose;

  if (!nav2_util::getCurrentPose(current_pose, *tf_buffer_,
    global_frame_, robot_base_frame_, transform_tolerance_)){
      return false;
  }

  pose = current_pose;
  return true;
}

void TrackingBase::speedLimitCallback(const nav2_msgs::msg::SpeedLimit::SharedPtr /*msg*/)
{
  // ControllerMap::iterator it;
  // for (it = controllers_.begin(); it != controllers_.end(); ++it) {
  //   it->second->setSpeedLimit(msg->speed_limit, msg->percentage);
  // }
}

void TrackingBase::updateObstacleContainerWithCostmapConverter()
{
  if (!costmap_converter_)
    return;
    
  //Get obstacles from costmap converter
  costmap_converter::ObstacleArrayConstPtr obstacles = costmap_converter_->getObstacles();
  if (!obstacles)
    return;

  for (std::size_t i=0; i<obstacles->obstacles.size(); ++i)
  {
    const costmap_converter_msgs::msg::ObstacleMsg* obstacle = &obstacles->obstacles.at(i);
    const geometry_msgs::msg::Polygon* polygon = &obstacle->polygon;

    if (polygon->points.size()==1 && obstacle->radius > 0) // Circle
    {
      obstacles_.push_back(ObstaclePtr(new CircularObstacle(polygon->points[0].x, polygon->points[0].y, obstacle->radius)));
    }
    else if (polygon->points.size()==1) // Point
    {
      obstacles_.push_back(ObstaclePtr(new PointObstacle(polygon->points[0].x, polygon->points[0].y)));
    }
    else if (polygon->points.size()==2) // Line
    {
      obstacles_.push_back(ObstaclePtr(new LineObstacle(polygon->points[0].x, polygon->points[0].y,
                                                        polygon->points[1].x, polygon->points[1].y )));
    }
    else if (polygon->points.size()>2) // Real polygon
    {
        PolygonObstacle* polyobst = new PolygonObstacle;
        for (std::size_t j=0; j<polygon->points.size(); ++j)
        {
            polyobst->pushBackVertex(polygon->points[j].x, polygon->points[j].y);
        }
        polyobst->finalizePolygon();
        obstacles_.push_back(ObstaclePtr(polyobst));
    }

    // Set velocity, if obstacle is moving
    if(!obstacles_.empty())
      obstacles_.back()->setCentroidVelocity(obstacles->obstacles[i].velocities, obstacles->obstacles[i].orientation);
  }
}

void TrackingBase::updateObstacleContainerWithCostmap()
{  
  // Add costmap obstacles if desired
  if (cfg_->obstacles.include_costmap_obstacles)
  {
    std::lock_guard<std::recursive_mutex> lock(*costmap_->getMutex());

    Eigen::Vector2d robot_orient = robot_pose_.orientationUnitVec();
    
    for (unsigned int i=0; i<costmap_->getSizeInCellsX()-1; ++i)
    {
      for (unsigned int j=0; j<costmap_->getSizeInCellsY()-1; ++j)
      {
        if (costmap_->getCost(i,j) == nav2_costmap_2d::LETHAL_OBSTACLE)
        {
          Eigen::Vector2d obs;
          costmap_->mapToWorld(i,j,obs.coeffRef(0), obs.coeffRef(1));
            
          // check if obstacle is interesting (e.g. not far behind the robot)
          Eigen::Vector2d obs_dir = obs-robot_pose_.position();
          if ( obs_dir.dot(robot_orient) < 0 && obs_dir.norm() > cfg_->obstacles.costmap_obstacles_behind_robot_dist  )
            continue;
            
          obstacles_.push_back(ObstaclePtr(new PointObstacle(obs)));
        }
      }
    }
  }
}


}  // namespace tracking_base
