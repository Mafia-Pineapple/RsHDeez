#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <iostream>

class GoalPublisher : public rclcpp::Node
{
public:
  GoalPublisher() : Node("goal_publisher")
  {
    publisher_ = this->create_publisher<geometry_msgs::msg::Point>("/drone/goal", 10);
    
    // Timer to prompt for goals every 5 seconds
    timer_ = this->create_wall_timer(
      std::chrono::seconds(5),
      std::bind(&GoalPublisher::promptForGoal, this));
      
    RCLCPP_INFO(this->get_logger(), "Goal Publisher node started");
    RCLCPP_INFO(this->get_logger(), "You can also publish goals manually:");
    RCLCPP_INFO(this->get_logger(), "ros2 topic pub /drone/goal geometry_msgs/msg/Point \"{x: 5.0, y: 3.0, z: 2.0}\"");
  }

private:
  void promptForGoal()
  {
    // Example predefined goals
    static int goal_index = 0;
    std::vector<std::array<double, 3>> goals = {
      {5.0, 5.0, 2.0},
      {-5.0, 5.0, 3.0},
      {-5.0, -5.0, 2.5},
      {5.0, -5.0, 2.0},
      {0.0, 0.0, 3.0}
    };
    
    geometry_msgs::msg::Point goal;
    goal.x = goals[goal_index][0];
    goal.y = goals[goal_index][1]; 
    goal.z = goals[goal_index][2];
    
    publisher_->publish(goal);
    
    RCLCPP_INFO(this->get_logger(), "Published goal %d: (%.1f, %.1f, %.1f)", 
                goal_index + 1, goal.x, goal.y, goal.z);
    
    goal_index = (goal_index + 1) % goals.size();
  }

  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  
  // Option 1: Automatic goal publishing
  if (argc > 1 && std::string(argv[1]) == "auto") {
    rclcpp::spin(std::make_shared<GoalPublisher>());
  }
  // Option 2: Manual goal input
  else {
    auto node = rclcpp::Node::make_shared("manual_goal_publisher");
    auto publisher = node->create_publisher<geometry_msgs::msg::Point>("/drone/goal", 10);
    
    std::cout << "\n=== Manual Goal Publisher ===" << std::endl;
    std::cout << "Enter goal coordinates (x y z) or 'q' to quit:" << std::endl;
    
    std::string input;
    while (rclcpp::ok()) {
      std::cout << "Goal (x y z): ";
      std::getline(std::cin, input);
      
      if (input == "q" || input == "quit") {
        break;
      }
      
      std::istringstream iss(input);
      double x, y, z;
      if (iss >> x >> y >> z) {
        geometry_msgs::msg::Point goal;
        goal.x = x;
        goal.y = y;
        goal.z = z;
        
        publisher->publish(goal);
        RCLCPP_INFO(node->get_logger(), "Published goal: (%.2f, %.2f, %.2f)", x, y, z);
      } else {
        std::cout << "Invalid input. Please enter three numbers: x y z" << std::endl;
      }
    }
  }
  
  rclcpp::shutdown();
  return 0;
}