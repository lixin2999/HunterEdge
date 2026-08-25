// Copyright 2026 HUNTER Development Team
#include <memory>

#include "vision_perception/vision_perception.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<vision_perception::VisionPerception>(rclcpp::NodeOptions());

  node->configure();
  node->activate();

  rclcpp::spin(node->get_node_base_interface());

  node->deactivate();
  node->cleanup();
  node->shutdown();

  rclcpp::shutdown();
  return 0;
}
