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

#ifndef TRACKING_BASE__TARGET_MANAGER_HPP_
#define TRACKING_BASE__TARGET_MANAGER_HPP_

#include <memory>
#include <string>
#include <mutex>
#include <deque>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <visualization_msgs/msg/marker.hpp>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_util/lifecycle_node.hpp"

namespace tracking_base
{
static double poseDistanceSq(const geometry_msgs::msg::Pose & p1, const geometry_msgs::msg::Pose & p2)
{
  double dx = p2.position.x - p1.position.x;
  double dy = p2.position.y - p1.position.y;
  return dx * dx + dy * dy;
}
/**
 * @brief target manager
 * the current goal on the blackboard
 */
class TargetManager
{
public:
  /**
   * @brief A constructor for tracking_base::TargetManager
   * @param xml_tag_name Name for the XML tag for this node
   * @param conf BT node configuration
   */
  TargetManager(rclcpp_lifecycle::LifecycleNode::SharedPtr node);
  bool getTarget(geometry_msgs::msg::PoseStamped& target);
  bool getRawTarget(geometry_msgs::msg::PoseStamped& target){ target = last_raw_goal_; }

protected:

private:
  /**
   * @brief Callback function for goal update topic
   * @param msg Shared pointer to geometry_msgs::msg::PoseStamped message
   */
  void callback_updated_goal(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

  geometry_msgs::msg::PoseStamped translatePoseByMode(const geometry_msgs::msg::PoseStamped & pose);

  bool isValid(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  geometry_msgs::msg::PoseStamped last_goal_received_, last_raw_goal_;
  geometry_msgs::msg::PoseStamped last_goal_transformed_;
  rclcpp::Time latest_timestamp_;

  unsigned char current_mode_ = 255;

  std::string global_frame_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::deque<geometry_msgs::msg::PoseStamped> historical_poses_;
  int max_pose_inuse_;
  double dist_sq_throttle_, overtime_;
  float distance_, keep_distance_;

  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
};

}  // namespace tracking_base

#endif  // TRACKING_BASE__TARGET_MANAGER_HPP_
