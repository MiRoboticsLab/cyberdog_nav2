#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include<ctime>
#include <random>
#include "std_srvs/srv/trigger.hpp"

class TopicPublisher01 : public rclcpp::Node
{
public:
  TopicPublisher01(std::string name) : Node(name)
  {
    command_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("tracking_pose", 10);
    timer_ = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&TopicPublisher01::timer_callback, this));
    target_pose_srv_ = this->create_service<std_srvs::srv::Trigger>(
    "target_pose", std::bind(&TopicPublisher01::chargepose, this, std::placeholders::_1, std::placeholders::_2));
  }

private:
  void func1()
  {
    message.pose.position.x=2;
    message.pose.position.y=2;
    message.pose.position.z=2;
    message.pose.orientation.x=1;
    message.pose.orientation.y=1;
    message.pose.orientation.z=1;
    message.pose.orientation.w=1;
  }

  void func2()
  {
    message.pose.position.x=4;
    message.pose.position.y=4;
    message.pose.position.z=2;
    message.pose.orientation.x=1;
    message.pose.orientation.y=1;
    message.pose.orientation.z=1;
    message.pose.orientation.w=1;
  }

  void timer_callback()
  {
    time_t timer=std::time(0);
    if (flag == 0) {
      func1();
    }
    if(flag == 1){
      func2();
    }
    message.header.stamp.sec=timer;
    message.header.frame_id="se";
    command_publisher_->publish(message);
  }
  void chargepose(const std_srvs::srv::Trigger_Request::SharedPtr request, std_srvs::srv::Trigger_Response::SharedPtr response)
  {
    request;
    response->success = true;
    if(flag == 0){
      flag = 1;
    }
    else{
      flag =0;
    }
  }

  rclcpp::TimerBase::SharedPtr timer_;
  geometry_msgs::msg::PoseStamped message;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr command_publisher_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr target_pose_srv_;
  bool flag = false;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TopicPublisher01>("topic_publisher");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}