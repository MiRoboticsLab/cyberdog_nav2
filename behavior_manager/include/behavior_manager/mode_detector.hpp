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

#ifndef BEHAVIOR_MANAGER__MODE_DETECTOR_HPP_
#define BEHAVIOR_MANAGER__MODE_DETECTOR_HPP_

#include <memory>
#include <string>
#include <deque>
#include <unordered_map>
#include <algorithm>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int8.hpp"
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "protocol/srv/motion_result_cmd.hpp"
#include "cyberdog_debug/backtrace.hpp"
#include "motion_action/motion_macros.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "cyberdog_common/cyberdog_toml.hpp"

namespace cyberdog
{
namespace algorithm
{
class ModeDetector
{
public:
  enum class Stage : uint8_t
  {
    kNormallyTracking,
    kAutonomouslyTracking,
    kStairJumping,
    kAbnorm,
  };
  enum class StairDetection : int8_t
  {
    kUpStair = 1,
    kNothing = 0,
    kDownStair = -1,
  };
  explicit ModeDetector(const rclcpp::Node::SharedPtr node)
  : node_(node)
  {
    std::string toml_file = ament_index_cpp::get_package_share_directory(
      "behavior_manager") + "/config/detect.toml";
    toml::value config;
    if (!cyberdog::common::CyberdogToml::ParseFile(toml_file, config)) {
      FATAL("Cannot parse %s", toml_file.c_str());
      exit(-1);
    }
    GET_TOML_VALUE(config, "diff_x_threashold", diff_x_threashold_);
    GET_TOML_VALUE(config, "diff_y_threashold", diff_y_threashold_);
    GET_TOML_VALUE(config, "detect_duration", detect_duration_);
    GET_TOML_VALUE(config, "pose_topic_name", pose_topic_name_);
    GET_TOML_VALUE(config, "filter_size", filter_size_);
    INFO("diff_x threashold: %f, diff_y threashold: %f", diff_x_threashold_, diff_y_threashold_);
    INFO("detect_duration: %d", detect_duration_);
    INFO("pose_topic_name: %s", pose_topic_name_.c_str());
    INFO("filter_size: %d", filter_size_);
    stair_detected_sub_ = node_->create_subscription<std_msgs::msg::Int8>(
      "elevation_mapping/stair_detected",
      rclcpp::SystemDefaultsQoS(),
      std::bind(
        &ModeDetector::HandleStairDetectionCallback,
        this, std::placeholders::_1));
    rclcpp::SensorDataQoS sub_qos;
    if (pose_topic_name_ == "tracking_pose") {
      sub_qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    } else if (pose_topic_name_ == "tracking_pose_transformed") {
      sub_qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
    } else {
      ERROR("Unknown Pose topic: %s", pose_topic_name_.c_str());
      exit(-1);
    }
    target_pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
      pose_topic_name_,
      sub_qos,
      std::bind(
        &ModeDetector::HandleTargetPoseCallback,
        this, std::placeholders::_1));
    filtered_pose_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(
      "pose_filtered", sub_qos);
    // std::thread{[this] {rclcpp::spin(node_);}}.detach();
  }
  ~ModeDetector() {}
  bool Init(
    std::function<void(bool)> do_stair_jump_func,
    std::function<void()> do_auto_tracking_func,
    std::function<void(bool)> do_normal_tracking_func
  )
  {
    do_stair_jump_func_ = do_stair_jump_func;
    do_auto_tracking_func_ = do_auto_tracking_func;
    do_normal_tracking_func_ = do_normal_tracking_func;
    return true;
  }
  void Launch(bool stair_detect = true, bool static_detect = false)
  {
    stair_detect_ = stair_detect;
    static_detect_ = static_detect;
  }
  void Reset()
  {
    last_static_ = false;
    stair_detection_ = static_cast<int8_t>(StairDetection::kNothing);
    // TODO(lijian): 目标静止检测相关的变量复位
    // target_first_get = false;
    // pose_queue_.clear();
    timestamp_.clear();
    pose_x_.clear();
    pose_y_.clear();
    pose_x_filter_.clear();
    pose_y_filter_.clear();
    first_pop_ = false;
  }

private:
  void HandleStairDetectionCallback(const std_msgs::msg::Int8::SharedPtr msg)
  {
    if (!stair_detect_) {
      return;
    }
    INFO_EXPRESSION(msg->data != stair_detection_, "Detect stair: %d", msg->data);
    stair_detection_ = msg->data;
    if (stair_detection_ == static_cast<int8_t>(StairDetection::kUpStair)) {
      do_stair_jump_func_(true);
    } else if (stair_detection_ == static_cast<int8_t>(StairDetection::kDownStair)) {
      do_stair_jump_func_(false);
    }
  }
  void HandleTargetPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    if (!static_detect_) {
      return;
    }
    bool target_static = CheckTargetStatic(msg);
    INFO("%s", target_static ? "Static" : "Moving");
    if (target_static == last_static_) {
      return;
    }
    last_static_ = target_static;
    if (target_static) {
      INFO("Target Checked Static");
      do_auto_tracking_func_();
    } else {
      INFO("Target Checked Moving");
      do_normal_tracking_func_(true);
    }
  }

  double Filter(std::deque<double> & filter, double value)
  {
    filter.push_back(value);
    if (filter.size() < filter_size_) {
      return value;
    }
    auto filter_sorted = filter;
    // INFO("before: [%f],[%f],[%f],[%f],[%f]",
    // filter[0], filter[1], filter[2], filter[3], filter[4]);
    std::sort(filter_sorted.begin(), filter_sorted.end());
    double filtered = filter_sorted.at((filter_size_ - 1) / 2);
    // INFO("%f", filtered);
    filter.pop_front();
    // INFO(" after: [%f],[%f],[%f],[%f]", filter[0], filter[1], filter[2], filter[3]);
    return filtered;
  }
  /**
   * @brief
   * 检测目标是否处于静止状态，判断依据：在设定时长内目标的位姿没有设定范围外的变化
   *
   * @param msg
   * @return true
   * @return false
   */
  bool CheckTargetStatic(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    if (!EnQueue(msg)) {
      INFO("Pose not enough");
      return false;
    }
    auto max_x = *std::max_element(pose_x_.begin(), pose_x_.end());
    auto min_x = *std::min_element(pose_x_.begin(), pose_x_.end());
    auto diff_x = max_x - min_x;
    auto max_y = *std::max_element(pose_y_.begin(), pose_y_.end());
    auto min_y = *std::min_element(pose_y_.begin(), pose_y_.end());
    auto diff_y = max_y - min_y;

    // INFO("diff_x: %f, diff_y: %f", diff_x, diff_y);
    if (diff_x > diff_x_threashold_ || diff_y > diff_y_threashold_) {
      return false;
    } else {
      return true;
    }
  }
  bool EnQueue(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    timestamp_.push_back(msg->header.stamp.sec);
    auto pose_x_filtered = Filter(pose_x_filter_, msg->pose.position.x);
    auto pose_y_filtered = Filter(pose_y_filter_, msg->pose.position.y);
    geometry_msgs::msg::PoseStamped pose;
    pose.header = msg->header;
    pose.pose.position.x = pose_x_filtered;
    pose.pose.position.y = pose_y_filtered;
    pose.pose.position.z = msg->pose.position.z;
    pose.pose.orientation = msg->pose.orientation;
    filtered_pose_pub_->publish(pose);
    pose_x_.push_back(pose_x_filtered);
    pose_y_.push_back(pose_y_filtered);
    // INFO("back: %d, front: %d", timestamp_.back(), timestamp_.front());
    if (timestamp_.back() - timestamp_.front() < detect_duration_ && !first_pop_) {
      return false;
    }
    first_pop_ = true;
    timestamp_.pop_front();
    pose_x_.pop_front();
    pose_y_.pop_front();
    return true;
  }
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr stair_detected_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr filtered_pose_pub_;
  geometry_msgs::msg::PoseStamped::SharedPtr current_pose_;
  Stage stage_;
  std::function<void(bool)> do_stair_jump_func_;
  std::function<void()> do_auto_tracking_func_;
  std::function<void(bool)> do_normal_tracking_func_;
  // std::deque<geometry_msgs::msg::PoseStamped> pose_queue_;
  std::deque<int> timestamp_;
  std::deque<double> pose_x_, pose_y_;
  std::deque<double> pose_x_filter_, pose_y_filter_;
  geometry_msgs::msg::PoseStamped target_first;
  geometry_msgs::msg::PoseStamped target_current;
  std::string pose_topic_name_;
  double target_first_timestamp = -1;
  double target_current_timestamp = -1;
  float diff_x_threashold_{0}, diff_y_threashold_{0};
  int detect_duration_{0};
  int filter_size_;
  int8_t stair_detection_{0};
  bool stair_detect_{false}, static_detect_{false};
  bool last_static_{false};
  // bool target_first_get = false;
  bool first_pop_{false};
};  // class ModeDetector
}  // namespace algorithm
}  // namespace cyberdog
#endif  // BEHAVIOR_MANAGER__MODE_DETECTOR_HPP_
