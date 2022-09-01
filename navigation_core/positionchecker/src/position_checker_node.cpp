#include "positionchecker/position_checker_node.hpp"

#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include <memory>
#include <string>
#include <vector>
using namespace std::chrono_literals;
namespace CYBERDOG_NAV {

PositionChecker::PositionChecker()
    : rclcpp::Node("PositionChecker"), looping_(false) {
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
      get_node_base_interface(), get_node_timers_interface());
  tf_buffer_->setCreateTimerInterface(timer_interface);
  tf_buffer_->setUsingDedicatedThread(true);
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  enable_service = create_service<SetBool>(
      "PoseEnable",
      std::bind(&PositionChecker::serviceCallback, this, std::placeholders::_1,
                std::placeholders::_2, std::placeholders::_3));
  pos_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      "dog_pose", rclcpp::SystemDefaultsQoS());
}

PositionChecker::~PositionChecker() {}

void PositionChecker::loop() {
  geometry_msgs::msg::PoseStamped pose_based_on_global_frame;
  std::string global_frame_, robot_base_frame_;
  rclcpp::WallRate r(500ms);
  while (true) {
    RCLCPP_WARN(get_logger(), "Lopping.");
    sleep(1);
    // if (!nav2_util::getCurrentPose(pose_based_on_global_frame, *tf_buffer_,
    //                                "map", "base_link")) {
    //   RCLCPP_WARN(
    //       get_logger(),
    //       "Failed to obtain current pose based on map coordinate system.");
    // } else {
    //   RCLCPP_INFO(get_logger(), "current pose (%f,%f)",
    //               pose_based_on_global_frame.pose.position.x,
    //               pose_based_on_global_frame.pose.position.y);

      pose_based_on_global_frame.pose.position.x = 0;
      pose_based_on_global_frame.pose.position.y = 0;
      pos_pub_->publish(pose_based_on_global_frame);
    // }
    r.sleep();
  }
}
void PositionChecker::serviceCallback(
    const std::shared_ptr<rmw_request_id_t>,
    const std::shared_ptr<SetBool::Request> request,
    std::shared_ptr<SetBool::Response> response) {
  RCLCPP_WARN(get_logger(), "serviceCallback.");
  if (request->data == true && !looping_) {
    looping_ = true;
    RCLCPP_WARN(get_logger(), "serviceCallback2.");

    loop_thread_ = std::make_shared<std::thread>(&PositionChecker::loop, this);
    RCLCPP_WARN(get_logger(), "serviceCallback3.");
    response->success = true;
  } else if (request->data == false) {
    RCLCPP_WARN(get_logger(), "serviceCallback4.");
    looping_ = false;
    loop_thread_->join();
    response->success = true;
  }
  response->success = false;
}

}  // namespace CYBERDOG_NAV
