#pragma once
#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <thread>
#include <atomic>

class Controller; // forward-declare

/**
 * Thin wrapper node that owns and spins the Controller internally.
 * This ensures existing launch files that only spin Quadcopter still
 * run the control loop.
 */
class Quadcopter : public rclcpp::Node {
public:
  explicit Quadcopter(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~Quadcopter();

private:
  std::shared_ptr<Controller> controller_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> inner_exec_;
  std::thread spin_thread_;
  std::atomic<bool> running_{false};
};
