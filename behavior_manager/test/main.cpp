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
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int8.hpp"
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "protocol/srv/motion_result_cmd.hpp"
#include "cyberdog_debug/backtrace.hpp"

class FakeTracking
{
public:
  explicit FakeTracking(const std::string & node_name)
  {
    node_ = std::make_shared<rclcpp::Node>(node_name);
    tracking_srv_ = node_->create_service<std_srvs::srv::SetBool>(
      "tracking_command",
      std::bind(&FakeTracking::HandleCommandSwitchCallback,
        this, std::placeholders::_1, std::placeholders::_2));
    std::thread{[this]{rclcpp::spin(node_);}}.detach();
  }
  ~FakeTracking(){}
private:
  void HandleCommandSwitchCallback(
    const std_srvs::srv::SetBool::Request::SharedPtr request,
    std_srvs::srv::SetBool::Response::SharedPtr response)
  {
    INFO("FakeTracking Receive: %d", request->data);
    response->success = true;
  }
  rclcpp::Node::SharedPtr node_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr tracking_srv_;
};  // class FakeTracking

class FakeElevation
{
public:
  explicit FakeElevation(const std::string & node_name)
  {
    node_ = std::make_shared<rclcpp::Node>(node_name);
    stair_jump_detected_srv_ = node_->create_service<std_srvs::srv::SetBool>(
      "elevation_mapping/stair_jump_detected",
      std::bind(&FakeElevation::HandleCommandSwitchCallback,
        this, std::placeholders::_1, std::placeholders::_2));
    std::thread{[this]{rclcpp::spin(node_);}}.detach();
  }
  ~FakeElevation(){}
private:
  void HandleCommandSwitchCallback(
    const std_srvs::srv::SetBool::Request::SharedPtr request,
    std_srvs::srv::SetBool::Response::SharedPtr response)
  {
    INFO("FakeElevation Receive: %d", request->data);
    response->success = true;
  }
  rclcpp::Node::SharedPtr node_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr stair_jump_detected_srv_;
};

class FakeMotion
{
public:
  explicit FakeMotion(const std::string & node_name)
  {
    node_ = std::make_shared<rclcpp::Node>(node_name);
    motion_srv_ = node_->create_service<protocol::srv::MotionResultCmd>(
      "motion_result_cmd",
      std::bind(&FakeMotion::HandleResultCmdCallback,
        this, std::placeholders::_1, std::placeholders::_2));
    std::thread{[this]{rclcpp::spin(node_);}}.detach();
  }
  ~FakeMotion(){}
private:
  void HandleResultCmdCallback(
    const protocol::srv::MotionResultCmd::Request::SharedPtr request,
    protocol::srv::MotionResultCmd::Response::SharedPtr response)
  {
    INFO("FakeMotion Receive: %d", request->motion_id);
    response->code = 0;
  }
  rclcpp::Node::SharedPtr node_;
  rclcpp::Service<protocol::srv::MotionResultCmd>::SharedPtr motion_srv_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("fake_nodes");
  auto fake_tracking = std::make_shared<FakeTracking>("fake_tracking");
  auto fake_elevation = std::make_shared<FakeElevation>("fake_elevation");
  auto fake_motion = std::make_shared<FakeMotion>("fake_motion");
  rclcpp::spin(node);
}