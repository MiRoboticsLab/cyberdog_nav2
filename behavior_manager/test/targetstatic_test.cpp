#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include<ctime>
#include <random>

class TopicPublisher01 : public rclcpp::Node
{
public:
  TopicPublisher01(std::string name) : Node(name)
  {
    command_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("tracking_pose", 10);
    timer_ = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&TopicPublisher01::timer_callback, this));
  }

private:
  void timer_callback()
  {
    time_t timer=std::time(0);
    std::random_device rd;  
    std::mt19937 gen(rd()); 
    std::uniform_real_distribution<> dis(1.0, 1.3005);
    geometry_msgs::msg::PoseStamped message;
    message.header.stamp.sec=timer;
    message.header.frame_id="se";
    message.pose.position.x=dis(gen);
    message.pose.position.y=2;
    message.pose.position.z=2;
    message.pose.orientation.x=1;
    message.pose.orientation.y=1;
    message.pose.orientation.z=1;
    message.pose.orientation.w=1;
    command_publisher_->publish(message);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr command_publisher_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TopicPublisher01>("topic_publisher");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}