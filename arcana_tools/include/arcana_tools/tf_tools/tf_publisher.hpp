#ifndef ARCANA_TF_PUBLISHER_HPP
#define ARCANA_TF_PUBLISHER_HPP

#include <chrono>
#include <geometry_msgs/msg/detail/transform_stamped__struct.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

using tf2_msgs::msg::TFMessage;
using namespace std::chrono;

namespace arcana {

struct TFPublisher : rclcpp::Node {

  TFPublisher() : rclcpp::Node("tf_pub") {

    timer_duration = milliseconds(declare_parameter("duration", 100));

    auto tf = geometry_msgs::msg::TransformStamped();
    tf.header.frame_id = declare_parameter("parent_frame", "map");
    tf.child_frame_id = declare_parameter("child_frame", "odom");
    tf.transform.translation.x = declare_parameter("dx", 0.0);
    tf.transform.translation.y = declare_parameter("dy", 0.0);
    tf.transform.translation.z = declare_parameter("dz", 0.0);
    tf.transform.rotation.x = declare_parameter("rx", 0.0);
    tf.transform.rotation.y = declare_parameter("ry", 0.0);
    tf.transform.rotation.z = declare_parameter("rz", 0.0);
    msg.transforms.push_back(tf);

    tf_pub = create_publisher<TFMessage>("/tf", 10);
    pub_timer = create_wall_timer(timer_duration, [this]() {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Publishing TF");
      msg.transforms[0].header.stamp = get_clock()->now();
      tf_pub->publish(msg);
    });
  }

protected:
  milliseconds timer_duration;
  TFMessage msg;
  rclcpp::Publisher<TFMessage>::SharedPtr tf_pub;
  rclcpp::TimerBase::SharedPtr pub_timer;
};

} // namespace arcana

#endif