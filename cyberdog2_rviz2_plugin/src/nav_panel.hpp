#ifndef NAV_PANEL_H
#define NAV_PANEL_H

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <ament_index_cpp/get_package_prefix.hpp>
#include <rviz_common/panel.hpp>
#include <rviz_common/config.hpp>

#include <QPainter>
#include <QLineEdit>
#include <QGroupBox>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QTimer>
#include <QPixmap>
#include <QPushButton>

#include <rclcpp/rclcpp.hpp>
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "protocol/action/navigation.hpp"
#include "nav_combo_box.hpp"
#include "nav_push_btn.hpp"

namespace cyberdog2_control_plugin
{
class NavControlPanel : public rviz_common::Panel
{
  using Navigation = protocol::action::Navigation;
  using GoalHandleNavigation = rclcpp_action::ClientGoalHandle<Navigation>;

  Q_OBJECT

public:
  NavControlPanel(QWidget * parent = 0);
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
} // namespace cyberdog2_control_plugin

#endif // NAV_PANEL_H
