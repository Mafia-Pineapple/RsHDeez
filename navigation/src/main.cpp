#include "rclcpp/rclcpp.hpp"
#include "quadcopter.h"
#include "decision_making.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto quadcopter = std::make_shared<Quadcopter>();
  auto decision = std::make_shared<DecisionMaking>(quadcopter);

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(quadcopter);
  executor.add_node(decision);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
