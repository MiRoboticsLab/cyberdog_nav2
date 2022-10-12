#include <chrono>
#include <cinttypes>
#include <functional>
#include <future>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "cyberdog_common/cyberdog_log.hpp"
#include "protocol/action/navigation.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace cyberdog
{
namespace algorithm
{

class NavActionClient : public rclcpp::Node
{
public:
  using Navigation = protocol::action::Navigation;
  using GoalHandleNavigation = rclcpp_action::ClientGoalHandle<Navigation>;

  explicit NavActionClient(const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions())
  : Node("nav_action_example", node_options)
  {
    this->client_ptr_ = rclcpp_action::create_client<Navigation>(
      this->get_node_base_interface(),
      this->get_node_graph_interface(),
      this->get_node_logging_interface(),
      this->get_node_waitables_interface(),
      "start_algo_task");
  }

  void send_goal()
  {
    using namespace std::placeholders;
    if (!this->client_ptr_) {
      ERROR("Action client not initialized");
    }

    if (!this->client_ptr_->wait_for_action_server(std::chrono::seconds(10))) {
      ERROR("Action server not available after waiting");
      return;
    }

    auto goal = Navigation::Goal();
    goal.nav_type = 1;
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.pose.orientation.w = 1;
    pose.pose.position.x = 0.5;
    goal.poses.push_back(pose);
    INFO("Sending goal");

    auto send_goal_options = rclcpp_action::Client<Navigation>::SendGoalOptions();
    send_goal_options.goal_response_callback =
      std::bind(&NavActionClient::goal_response_callback, this, _1);
    send_goal_options.feedback_callback =
      std::bind(&NavActionClient::feedback_callback, this, _1, _2);
    send_goal_options.result_callback =
      std::bind(&NavActionClient::result_callback, this, _1);
    auto goal_handle_future = this->client_ptr_->async_send_goal(goal, send_goal_options);
  }

private:
  rclcpp_action::Client<Navigation>::SharedPtr client_ptr_;

  void goal_response_callback(GoalHandleNavigation::SharedPtr goal_handle)
  {
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
  }

  void feedback_callback(
    GoalHandleNavigation::SharedPtr,
    const std::shared_ptr<const Navigation::Feedback> feedback)
  {
  }

  void result_callback(const GoalHandleNavigation::WrappedResult & result)
  {
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
        return;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
        return;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
        return;
    }
  }
};  // class MinimalActionClient

}  // namespace algorithm
}  // namespace cyberdog

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto action_client = std::make_shared<::cyberdog::algorithm::NavActionClient>();
  action_client->send_goal();
  rclcpp::shutdown();
  return 0;
}
