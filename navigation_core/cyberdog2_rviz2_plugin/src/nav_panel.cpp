#include "nav_panel.hpp"

namespace cyberdog2_control_plugin
{

NavControlPanel::NavControlPanel(QWidget * parent)
: rviz_common::Panel(parent)
{
  nav_client_node_ = std::make_shared<rclcpp::Node>("cyberdog_nav_client");
  client_ptr_ = rclcpp_action::create_client<Navigation>(
    nav_client_node_,
    "fibonacci");
  goals_subscriber_ = nav_client_node_->create_subscription<geometry_msgs::msg::PoseStamped>(
    "goal_pose", rclcpp::SystemDefaultsQoS(),
    std::bind(&NavControlPanel::nav_goals_callback, this, std::placeholders::_1));
  spin_thread = std::thread(
    [this]() {
      rclcpp::spin(nav_client_node_);
    });
  //interface
  QVBoxLayout * layout = new QVBoxLayout;
  QVBoxLayout * nav_layout = new QVBoxLayout;
  nav_combo_ = new NavComboBox(this);
  nav_layout->addLayout(nav_combo_, Qt::AlignLeft);
  layout->addLayout(nav_layout);
  QVBoxLayout * action_layout = new QVBoxLayout;
  nav_btn_ = new NavPushBtn(this);
  action_layout->addLayout(nav_btn_, Qt::AlignLeft);
  layout->addLayout(action_layout);
  setLayout(layout);

  connect(nav_combo_, SIGNAL(valueChanged(int)), SLOT(set_nav_goal(int)));
  connect(nav_btn_, &NavPushBtn::clicked, [this](void) {send_nav_action();});
}

void NavControlPanel::save(rviz_common::Config config) const
{
  rviz_common::Panel::save(config);
}

// Load all configuration data for this panel from the given Config object.
void NavControlPanel::load(const rviz_common::Config & config)
{
  rviz_common::Panel::load(config);
}

void NavControlPanel::send_goal(Navigation::Goal & goal_msg)
{
  if (!client_ptr_->wait_for_action_server()) {
    RCLCPP_ERROR(nav_client_node_->get_logger(), "Action server not available after waiting");
    rclcpp::shutdown();
  }
  auto send_goal_options = rclcpp_action::Client<Navigation>::SendGoalOptions();
  send_goal_options.goal_response_callback =
    std::bind(&NavControlPanel::goal_response_callback, this, std::placeholders::_1);
  send_goal_options.feedback_callback =
    std::bind(
    &NavControlPanel::feedback_callback, this, std::placeholders::_1,
    std::placeholders::_2);
  send_goal_options.result_callback =
    std::bind(&NavControlPanel::result_callback, this, std::placeholders::_1);
  client_ptr_->async_send_goal(goal_msg, send_goal_options);
}

void NavControlPanel::goal_response_callback(
  std::shared_future<GoalHandleNavigation::SharedPtr> future)
{
  (void) future;
}

void NavControlPanel::feedback_callback(
  GoalHandleNavigation::SharedPtr,
  const std::shared_ptr<const Navigation::Feedback> feedback)
{
  (void) feedback;
}

void NavControlPanel::result_callback(const GoalHandleNavigation::WrappedResult & result)
{
  (void) result;
}

void NavControlPanel::set_nav_goal(int goal_id)
{
  goal_method_ = goal_id;
  nav_btn_->setEnabled(true);
}

void NavControlPanel::send_nav_action()
{
  receive_goals = false;
  switch (goal_method_) {
    case (1):
      {
        receive_goals = true;
        nav_btn_->setEnabled(false);
      } break;
    case (2):
      {
      }
      break;
    case (3):
      {
        Navigation::Goal goal;
        goal.poses.resize(0);
        goal.nav_type = protocol::action::Navigation::Goal::NAVIGATION_GOAL_TYPE_MAPPING;
        send_goal(goal);
        nav_btn_->setEnabled(false);
      } break;
    case (4):
      {
        Navigation::Goal goal;
        goal.poses.resize(0);
        goal.nav_type = protocol::action::Navigation::Goal::NAVIGATION_GOAL_TYPE_STOP_MAPPING;
        send_goal(goal);
        nav_btn_->setEnabled(false);
      } break;
    default:
      break;
  }
}

void NavControlPanel::nav_goals_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  if (receive_goals) {
    printf("send goals start!\n");
    Navigation::Goal goal;
    goal.nav_type = protocol::action::Navigation::Goal::NAVIGATION_GOAL_TYPE_AB;
    goal.poses.resize(1);
    goal.poses[0] = *msg;
    send_goal(goal);
    nav_btn_->setEnabled(true);
    printf("send goals ok!\n");
  }
}

bool NavControlPanel::event(QEvent * event)
{
  (void) event;
  return false;
}

} // namespace cyberdog2_control_plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(cyberdog2_control_plugin::NavControlPanel, rviz_common::Panel)
