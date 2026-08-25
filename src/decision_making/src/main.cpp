// Copyright 2026 HUNTER Development Team
#include <memory>

#include "decision_making/decision_making.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<decision_making::DecisionMaking>(rclcpp::NodeOptions());

  node->configure();
  node->activate();

  rclcpp::spin(node->get_node_base_interface());

  node->deactivate();
  node->cleanup();
  node->shutdown();

  rclcpp::shutdown();
  return 0;
}
