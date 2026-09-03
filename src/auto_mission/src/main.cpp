// Copyright 2026 HUNTER Development Team
// auto_mission_node 入口
#include "auto_mission/auto_mission_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<auto_mission::AutoMissionNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
