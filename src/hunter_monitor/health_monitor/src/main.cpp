// Copyright 2026 HUNTER Development Team
#include <memory>

#include "health_monitor/health_monitor.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<health_monitor::HealthMonitor>(rclcpp::NodeOptions());
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
