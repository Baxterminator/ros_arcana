#include "arcana_tools/tf_tools/tf_analyzer.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<arcana::TFAnalyzer>();
  while (rclcpp::ok())
    rclcpp::spin_some(node);
  rclcpp::shutdown();
}