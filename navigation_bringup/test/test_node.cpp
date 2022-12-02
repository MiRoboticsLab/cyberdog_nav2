#include <chrono>
#include <memory>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "cyberdog_common/cyberdog_log.hpp"

class FrameListener : public rclcpp::Node
{
public:
  FrameListener()
  : Node("test_node")
  {
    tf_buffer_ =
      std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ =
      std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(1000), std::bind(&FrameListener::on_timer, this));
  }

private:
  void on_timer()
  {
    geometry_msgs::msg::TransformStamped t;
    geometry_msgs::msg::TransformStamped t1;
    geometry_msgs::msg::TransformStamped t2;
    geometry_msgs::msg::TransformStamped t3;
    geometry_msgs::msg::TransformStamped t4;
    geometry_msgs::msg::TransformStamped t5;
    geometry_msgs::msg::TransformStamped t6;
    geometry_msgs::msg::TransformStamped t7;
    geometry_msgs::msg::TransformStamped t8;
    geometry_msgs::msg::TransformStamped t9;
    geometry_msgs::msg::TransformStamped t10;
    geometry_msgs::msg::TransformStamped t11;
    geometry_msgs::msg::TransformStamped t12;

    t = tf_buffer_->lookupTransform(
      "laser_frame", "base_link",
      tf2::TimePointZero);
    INFO(
      "tf2_node_base_to_lidar x: %f, y: %f, z: %f, x: %f, y: %f, z: %f, w: %f",
      t.transform.translation.x, t.transform.translation.y, t.transform.translation.z,
      t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z,
      t.transform.rotation.w);

    t1 = tf_buffer_->lookupTransform(
      "camera_link", "base_link",
      tf2::TimePointZero);
    INFO(
      "tf2_node_base_to_camera x: %f, y: %f, z: %f, x: %f, y: %f, z: %f, w: %f",
      t1.transform.translation.x, t1.transform.translation.y, t1.transform.translation.z,
      t1.transform.rotation.x, t1.transform.rotation.y, t1.transform.rotation.z,
      t1.transform.rotation.w);

    t2 = tf_buffer_->lookupTransform(
      "left_head", "base_link",
      tf2::TimePointZero);
    INFO(
      "tf2_node_base_to_tof_left_head x: %f, y: %f, z: %f, x: %f, y: %f, z: %f, w: %f",
      t2.transform.translation.x, t2.transform.translation.y, t2.transform.translation.z,
      t2.transform.rotation.x, t2.transform.rotation.y, t2.transform.rotation.z,
      t2.transform.rotation.w);

    t3 = tf_buffer_->lookupTransform(
      "right_head", "base_link",
      tf2::TimePointZero);
    INFO(
      "tf2_node_base_to_tof_right_head x: %f, y: %f, z: %f, x: %f, y: %f, z: %f, w: %f",
      t3.transform.translation.x, t3.transform.translation.y, t3.transform.translation.z,
      t3.transform.rotation.x, t3.transform.rotation.y, t3.transform.rotation.z,
      t3.transform.rotation.w);

    t4 = tf_buffer_->lookupTransform(
      "left_rear", "base_link",
      tf2::TimePointZero);
    INFO(
      "tf2_node_base_to_tof_left_rear x: %f, y: %f, z: %f, x: %f, y: %f, z: %f, w: %f",
      t4.transform.translation.x, t4.transform.translation.y, t4.transform.translation.z,
      t4.transform.rotation.x, t4.transform.rotation.y, t4.transform.rotation.z,
      t4.transform.rotation.w);

    t5 = tf_buffer_->lookupTransform(
      "right_rear", "base_link",
      tf2::TimePointZero);
    INFO(
      "tf2_node_base_to_tof_right_rear x: %f, y: %f, z: %f, x: %f, y: %f, z: %f, w: %f",
      t5.transform.translation.x, t5.transform.translation.y, t5.transform.translation.z,
      t5.transform.rotation.x, t5.transform.rotation.y, t5.transform.rotation.z,
      t5.transform.rotation.w);

    t6 = tf_buffer_->lookupTransform(
      "uwb", "base_link",
      tf2::TimePointZero);
    INFO(
      "tf2_node_base_to_uwb x: %f, y: %f, z: %f, x: %f, y: %f, z: %f, w: %f",
      t6.transform.translation.x, t6.transform.translation.y, t6.transform.translation.z,
      t6.transform.rotation.x, t6.transform.rotation.y, t6.transform.rotation.z,
      t6.transform.rotation.w);

    t7 = tf_buffer_->lookupTransform(
      "head_tof", "uwb",
      tf2::TimePointZero);
    INFO(
      "tf2_node_uwb_to_head_tof x: %f, y: %f, z: %f, x: %f, y: %f, z: %f, w: %f",
      t7.transform.translation.x, t7.transform.translation.y, t7.transform.translation.z,
      t7.transform.rotation.x, t7.transform.rotation.y, t7.transform.rotation.z,
      t7.transform.rotation.w);

    t8 = tf_buffer_->lookupTransform(
      "head_uwb", "uwb",
      tf2::TimePointZero);
    INFO(
      "tf2_node_uwb_to_head_uwb x: %f, y: %f, z: %f, x: %f, y: %f, z: %f, w: %f",
      t8.transform.translation.x, t8.transform.translation.y, t8.transform.translation.z,
      t8.transform.rotation.x, t8.transform.rotation.y, t8.transform.rotation.z,
      t8.transform.rotation.w);

    t9 = tf_buffer_->lookupTransform(
      "rear_uwb", "uwb",
      tf2::TimePointZero);
    INFO(
      "tf2_node_uwb_to_rear_uwb x: %f, y: %f, z: %f, x: %f, y: %f, z: %f, w: %f",
      t9.transform.translation.x, t9.transform.translation.y, t9.transform.translation.z,
      t9.transform.rotation.x, t9.transform.rotation.y, t9.transform.rotation.z,
      t9.transform.rotation.w);

    t10 = tf_buffer_->lookupTransform(
      "rear_tof", "uwb",
      tf2::TimePointZero);
    INFO(
      "tf2_node_uwb_to_rear_tof x: %f, y: %f, z: %f, x: %f, y: %f, z: %f, w: %f",
      t10.transform.translation.x, t10.transform.translation.y, t10.transform.translation.z,
      t10.transform.rotation.x, t10.transform.rotation.y, t10.transform.rotation.z,
      t10.transform.rotation.w);

    t11 = tf_buffer_->lookupTransform(
      "odom", "map",
      tf2::TimePointZero);
    INFO(
      "tf2_node_map_to_odom x: %f, y: %f, z: %f, x: %f, y: %f, z: %f, w: %f",
      t11.transform.translation.x, t11.transform.translation.y, t11.transform.translation.z,
      t11.transform.rotation.x, t11.transform.rotation.y, t11.transform.rotation.z,
      t11.transform.rotation.w);

    t12 = tf_buffer_->lookupTransform(
      "vodom", "map",
      tf2::TimePointZero);
    INFO(
      "tf2_node_map_to_vodom x: %f, y: %f, z: %f, x: %f, y: %f, z: %f, w: %f",
      t12.transform.translation.x, t12.transform.translation.y, t12.transform.translation.z,
      t12.transform.rotation.x, t12.transform.rotation.y, t12.transform.rotation.z,
      t12.transform.rotation.w);
  }

  rclcpp::TimerBase::SharedPtr timer_{nullptr};
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FrameListener>());
  rclcpp::shutdown();
  return 0;
}
