// Copyright 2026 HUNTER Development Team
#include <memory>

#include "ch10x_driver/ch10x_driver.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<ch10x_driver::Ch10xDriver>(rclcpp::NodeOptions());

  // 独立运行：手动触发生命周期（也可交由 lifecycle manager 统一管理）
  node->configure();
  node->activate();

  rclcpp::spin(node->get_node_base_interface());

  node->deactivate();
  node->cleanup();
  node->shutdown();

  rclcpp::shutdown();
  return 0;
}
