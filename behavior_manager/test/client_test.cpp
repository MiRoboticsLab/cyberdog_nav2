#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "behavior_manager/executor_auto_tracking.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("node_01");
  auto executor_auto_tracking_ = std::make_shared<cyberdog::algorithm::ExecutorAutoTracking>(node);
  executor_auto_tracking_->Execute(true);
  rclcpp::spin(node);
}