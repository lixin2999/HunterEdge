// Copyright 2026 HUNTER Development Team
#include <memory>

#include "lidar_perception/lidar_perception.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<lidar_perception::LidarPerception>(rclcpp::NodeOptions());

  node->configure();
  node->activate();

  rclcpp::spin(node->get_node_base_interface());

  node->deactivate();
  node->cleanup();
  node->shutdown();

  rclcpp::shutdown();
  return 0;
}
