#include "arcana_tools/tf_tools/tf_publisher.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<arcana::TFPublisher>();
  while (rclcpp::ok())
    rclcpp::spin_some(node);
  rclcpp::shutdown();
}