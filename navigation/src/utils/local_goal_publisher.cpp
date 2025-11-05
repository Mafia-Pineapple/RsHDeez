// src/local_goal_publisher.cpp
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>

#include <chrono>
#include <iostream>
#include <sstream>
#include <cmath>

using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandleNTP  = rclcpp_action::ClientGoalHandle<NavigateToPose>;

class LocalGoalPublisher : public rclcpp::Node {
public:
  LocalGoalPublisher()
  : Node("local_goal_publisher"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    // Parameters
    local_frame_  = this->declare_parameter<std::string>("local_frame", "odom");
    base_frame_   = this->declare_parameter<std::string>("base_frame",  "base_link");
    action_name_  = this->declare_parameter<std::string>("action_name", "navigate_to_pose");

    // Action client
    client_ = rclcpp_action::create_client<NavigateToPose>(this, action_name_);

    // Timers: one to check server + read stdin, one for throttled server wait logs
    stdin_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&LocalGoalPublisher::pollStdinAndMaybeSend, this));

    // Topic interface for scripted goals
    cmd_sub_ = this->create_subscription<geometry_msgs::msg::Vector3>(
        "/local_goal_cmd", 10,
        std::bind(&LocalGoalPublisher::onCmdMsg, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(),
      "LocalGoalPublisher ready.\n"
      " - Frames: local='%s', base='%s'\n"
      " - Action: '%s'\n"
      " - Topic : /local_goal_cmd (Vector3: x=dx[m], y=dy[m], z=dyaw[deg])\n"
      " - STDIN : type 'dx dy dyaw_deg' (e.g. '10 0 0')",
      local_frame_.c_str(), base_frame_.c_str(), action_name_.c_str());
  }

private:
  // === Input paths ===
  void pollStdinAndMaybeSend() {
    // Don’t block if action server not up yet
    if (!waitForServerOnce()) return;

    // Non-blocking read: only read when something is ready on stdin
    // (Simple heuristic: std::cin.rdbuf()->in_avail(); works for piped input; for TTY we just prompt)
    static bool prompted = false;
    if (!prompted) {
      std::cout << "> (dx dy dyaw_deg) ";
      std::cout.flush();
      prompted = true;
    }

    // If user pressed Enter, getline returns a line
    if (!std::cin.good()) return;
    if (!std::cin.rdbuf()->in_avail()) return;

    std::string line;
    std::getline(std::cin, line);
    prompted = false;
    if (line.empty()) return;

    double dx, dy, dyaw_deg;
    std::istringstream iss(line);
    if (!(iss >> dx >> dy >> dyaw_deg)) {
      std::cout << "Format: dx dy dyaw_deg (e.g. '10 0 0')\n";
      return;
    }
    sendRelativeGoal(dx, dy, dyaw_deg);
  }

  void onCmdMsg(const geometry_msgs::msg::Vector3::SharedPtr msg) {
    if (!waitForServerOnce()) return;
    sendRelativeGoal(msg->x, msg->y, msg->z);
  }

  // === Core ===
  void sendRelativeGoal(double dx, double dy, double dyaw_deg) {
    // TF: local_frame -> base_frame
    geometry_msgs::msg::TransformStamped local_T_base_msg;
    try {
      local_T_base_msg = tf_buffer_.lookupTransform(local_frame_, base_frame_, tf2::TimePointZero);
    } catch (const tf2::TransformException &ex) {
      RCLCPP_WARN(get_logger(), "TF lookup %s->%s failed: %s",
                  local_frame_.c_str(), base_frame_.c_str(), ex.what());
      return;
    }

    tf2::Transform local_T_base;
    tf2::fromMsg(local_T_base_msg.transform, local_T_base);

    // Build relative transform in base frame
    tf2::Quaternion q_rel;
    q_rel.setRPY(0.0, 0.0, deg2rad(dyaw_deg));
    tf2::Transform base_T_goal_rel(q_rel, tf2::Vector3(dx, dy, 0.0));

    // Compose: local->goal = local->base * base->goal_rel
    tf2::Transform local_T_goal = local_T_base * base_T_goal_rel;

    // Convert to PoseStamped in local frame
    geometry_msgs::msg::PoseStamped goal;
    goal.header.stamp = now();
    goal.header.frame_id = local_frame_;
    goal.pose.position.x = local_T_goal.getOrigin().x();
    goal.pose.position.y = local_T_goal.getOrigin().y();
    goal.pose.position.z = local_T_goal.getOrigin().z();
    goal.pose.orientation = tf2::toMsg(local_T_goal.getRotation());

    // Build action goal
    NavigateToPose::Goal nav_goal;
    nav_goal.pose = goal;

    auto opts = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    opts.feedback_callback =
      [this](GoalHandleNTP::SharedPtr, const std::shared_ptr<const NavigateToPose::Feedback> fb) {
        // Distance and time_remaining fields may be NaN early on; guard prints
        if (std::isfinite(fb->distance_remaining)) {
          RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                               "Feedback: dist=%.2f m", fb->distance_remaining);
        }
      };
    opts.result_callback =
      [this](const GoalHandleNTP::WrappedResult &res) {
        switch (res.code) {
          case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(get_logger(), "NavigateToPose SUCCEEDED");
            break;
          case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_WARN(get_logger(), "NavigateToPose ABORTED");
            break;
          case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_WARN(get_logger(), "NavigateToPose CANCELED");
            break;
          default:
            RCLCPP_WARN(get_logger(), "NavigateToPose finished with unknown result");
            break;
        }
      };

    client_->async_send_goal(nav_goal, opts);
    RCLCPP_INFO(get_logger(), "Sent local goal in '%s': dx=%.2f dy=%.2f dyaw=%.1f°",
                local_frame_.c_str(), dx, dy, dyaw_deg);
  }

  bool waitForServerOnce() {
    if (client_->wait_for_action_server(std::chrono::milliseconds(10))) return true;
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 3000,
                         "Waiting for '%s' action server...", action_name_.c_str());
    return false;
  }

  static inline double deg2rad(double d) { return d * M_PI / 180.0; }

  // Members
  rclcpp_action::Client<NavigateToPose>::SharedPtr client_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::TimerBase::SharedPtr stdin_timer_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr cmd_sub_;

  std::string local_frame_;
  std::string base_frame_;
  std::string action_name_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LocalGoalPublisher>();
  rclcpp::executors::MultiThreadedExecutor exec;  // avoids blocking timers/subs
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
