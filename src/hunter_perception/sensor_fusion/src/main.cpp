// Copyright 2026 HUNTER Development Team
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_fusion/sensor_fusion.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<sensor_fusion::SensorFusion>(rclcpp::NodeOptions());
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
