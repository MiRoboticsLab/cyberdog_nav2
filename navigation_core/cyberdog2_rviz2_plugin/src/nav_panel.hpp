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


#ifndef NAV_PANEL_HPP_
#define NAV_PANEL_HPP_

#include <memory>       // NOLINT
#include <QPainter>     // NOLINT
#include <QLineEdit>    // NOLINT
#include <QGroupBox>    // NOLINT
#include <QVBoxLayout>  // NOLINT
#include <QHBoxLayout>  // NOLINT
#include <QLabel>       // NOLINT
#include <QTimer>       // NOLINT
#include <QPixmap>      // NOLINT
#include <QPushButton>  // NOLINT

#include <rclcpp/rclcpp.hpp>                // NOLINT
#include <rclcpp_action/rclcpp_action.hpp>  // NOLINT
#include <std_srvs/srv/set_bool.hpp>        // NOLINT

#include <ament_index_cpp/get_package_share_directory.hpp>  // NOLINT
#include <ament_index_cpp/get_package_prefix.hpp>           // NOLINT
#include <rviz_common/panel.hpp>                      // NOLINT
#include <rviz_common/config.hpp>                     // NOLINT

#include "rclcpp_components/register_node_macro.hpp"  // NOLINT
#include "geometry_msgs/msg/pose_stamped.hpp"         // NOLINT
#include "protocol/action/navigation.hpp"             // NOLINT
#include "nav_combo_box.hpp"                          // NOLINT
#include "nav_push_btn.hpp"                           // NOLINT

namespace cyberdog2_control_plugin
{
class NavControlPanel : public rviz_common::Panel
{
  using Navigation = protocol::action::Navigation;
  using GoalHandleNavigation = rclcpp_action::ClientGoalHandle<Navigation>;

  Q_OBJECT

public:
  NavControlPanel(QWidget * parent = 0);     // NOLINT
  virtual void load(const rviz_common::Config & config);
  virtual void save(rviz_common::Config config) const;

private:
  void send_goal(Navigation::Goal & goal_msg);
  void goal_response_callback(std::shared_future<GoalHandleNavigation::SharedPtr> future);
  void feedback_callback(
    GoalHandleNavigation::SharedPtr,
    const std::shared_ptr<const Navigation::Feedback> feedback);
  void result_callback(const GoalHandleNavigation::WrappedResult & result);

protected Q_SLOTS:
  void set_nav_goal(int goal_id);
  void send_nav_action();
  void nav_goals_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

protected:
  bool event(QEvent * event);

private:
  std::shared_ptr<rclcpp::Node> nav_client_node_;
  rclcpp_action::Client<Navigation>::SharedPtr client_ptr_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goals_subscriber_;
  std::thread spin_thread;

  NavComboBox * nav_combo_;
  NavPushBtn * nav_btn_;

  int goal_method_ = 0;
  bool receive_goals = false;
};
}  //  namespace cyberdog2_control_plugin

#endif  // NAV_PANEL_HPP_
