#include "quadcopter.h"
#include "controller.h"

Quadcopter::Quadcopter(const rclcpp::NodeOptions & options)
: rclcpp::Node("quadcopter", options)
{
  // Construct the controller node (shares the same process)
  controller_ = std::make_shared<Controller>();

  // Spin it on an internal executor thread so even if your main only spins
  // Quadcopter, the controller's timers/subscriptions still run.
  inner_exec_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
  inner_exec_->add_node(controller_);
  running_.store(true);
  spin_thread_ = std::thread([this](){
    while (rclcpp::ok() && running_.load()) {
      inner_exec_->spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
  });
}

Quadcopter::~Quadcopter()
{
  running_.store(false);
  if (inner_exec_) {
    inner_exec_->cancel();
  }
  if (spin_thread_.joinable()) {
    spin_thread_.join();
  }
  if (inner_exec_ && controller_) {
    inner_exec_->remove_node(controller_);
  }
}
