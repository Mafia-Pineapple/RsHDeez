#include "rclcpp/rclcpp.hpp"
#include "quadcopter.h"
#include "decision_making.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    RCLCPP_INFO(rclcpp::get_logger("main"), "Starting Quadcopter with Decision Making");
    
    // Create the quadcopter node
    auto quadcopter = std::make_shared<Quadcopter>();
    
    // Create the decision making node (it needs access to quadcopter)
    auto decision_making = std::make_shared<DecisionMaking>(quadcopter);
    
    // Use MultiThreadedExecutor to run both nodes
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(quadcopter);
    executor.add_node(decision_making);
    
    RCLCPP_INFO(rclcpp::get_logger("main"), "Both nodes initialized - starting execution");
    RCLCPP_INFO(rclcpp::get_logger("main"), "Quadcopter is in WANDERING mode");
    RCLCPP_INFO(rclcpp::get_logger("main"), "Publish to /thermal/detections to trigger investigation");
    
    executor.spin();
    
    rclcpp::shutdown();
    return 0;
}