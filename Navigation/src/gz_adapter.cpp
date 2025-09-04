#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <ros_gz_interfaces/srv/set_entity_linear_velocity.hpp>
#include <ros_gz_interfaces/srv/get_entity_state.hpp>
#include <ros_gz_interfaces/msg/entity.hpp>

using namespace std::chrono_literals;

class GzAdapter : public rclcpp::Node {
public:
  GzAdapter() : Node("gz_adapter")
  {
    // Parameters
    declare_parameter<std::string>("world", "default");
    declare_parameter<std::string>("model_name", "quad");

    world_ = get_parameter("world").as_string();
    model_name_ = get_parameter("model_name").as_string();

    // Gazebo Sim services
    set_vel_client_ = create_client<ros_gz_interfaces::srv::SetEntityLinearVelocity>(
      "/world/" + world_ + "/set_entity_linear_velocity");
    get_state_client_ = create_client<ros_gz_interfaces::srv::GetEntityState>(
      "/world/" + world_ + "/get_entity_state");

    // ROS I/O
    sub_cmd_ = create_subscription<geometry_msgs::msg::Twist>(
      "drone/cmd_vel", 10, [this](const geometry_msgs::msg::Twist& tw){ last_cmd_ = tw; });
    pub_odom_ = create_publisher<nav_msgs::msg::Odometry>("drone/gt_odom", 10);

    timer_ = create_wall_timer(20ms, [this]{ tick(); });
    RCLCPP_INFO(get_logger(), "GzAdapter up (world='%s', model='%s')",
                world_.c_str(), model_name_.c_str());
  }

private:
  void tick()
  {
    // 1) Send linear velocity to model
    if (set_vel_client_->service_is_ready()) {
      auto req = std::make_shared<ros_gz_interfaces::srv::SetEntityLinearVelocity::Request>();
      req->entity.name = model_name_;
      req->entity.type = ros_gz_interfaces::msg::Entity::MODEL;
      req->linear_velocity.x = last_cmd_.linear.x;
      req->linear_velocity.y = last_cmd_.linear.y;
      req->linear_velocity.z = last_cmd_.linear.z;
      (void)set_vel_client_->async_send_request(req);
    }

    // 2) Query pose and publish /drone/gt_odom
    if (get_state_client_->service_is_ready()) {
      auto req = std::make_shared<ros_gz_interfaces::srv::GetEntityState::Request>();
      req->entity.name = model_name_;
      req->entity.type = ros_gz_interfaces::msg::Entity::MODEL;

      auto fut = get_state_client_->async_send_request(req);
      if (fut.wait_for(std::chrono::milliseconds(5)) != std::future_status::ready) return;
      auto res = fut.get();
      if (!res->success) return;

      nav_msgs::msg::Odometry odom;
      odom.header.stamp = now();
      odom.header.frame_id = "map";
      odom.child_frame_id  = "base_link";
      odom.pose.pose.position = res->state.pose.position;
      odom.pose.pose.orientation = res->state.pose.orientation;
      pub_odom_->publish(odom);
    }
  }

  // Params
  std::string world_, model_name_;

  // ROS-GZ services
  rclcpp::Client<ros_gz_interfaces::srv::SetEntityLinearVelocity>::SharedPtr set_vel_client_;
  rclcpp::Client<ros_gz_interfaces::srv::GetEntityState>::SharedPtr          get_state_client_;

  // ROS I/O
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr      pub_odom_;
  rclcpp::TimerBase::SharedPtr                               timer_;
  geometry_msgs::msg::Twist last_cmd_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GzAdapter>());
  rclcpp::shutdown();
  return 0;
}
