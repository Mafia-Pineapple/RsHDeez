#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float64.hpp>
#include <deque>

class AglParser : public rclcpp::Node {
public:
  AglParser() : Node("agl_parser") {
    // Declare parameters
    this->declare_parameter("filter_invalid", true);
    this->declare_parameter("min_valid_range", 0.05);
    this->declare_parameter("max_valid_range", 95.0);
    this->declare_parameter("median_filter_size", 5);
    this->declare_parameter("use_odom_fallback", true);
    this->declare_parameter("ground_level_estimate", 0.0);  // Will be auto-detected
    
    filter_invalid_ = this->get_parameter("filter_invalid").as_bool();
    min_valid_range_ = this->get_parameter("min_valid_range").as_double();
    max_valid_range_ = this->get_parameter("max_valid_range").as_double();
    median_filter_size_ = this->get_parameter("median_filter_size").as_int();
    use_odom_fallback_ = this->get_parameter("use_odom_fallback").as_bool();
    ground_level_ = this->get_parameter("ground_level_estimate").as_double();
    
    // Subscribe to the LaserScan from the AGL sensor
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/agl", 10,
        std::bind(&AglParser::scanCallback, this, std::placeholders::_1));
    
    // Subscribe to odometry for fallback
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odometry", 10,
        std::bind(&AglParser::odomCallback, this, std::placeholders::_1));
    
    // Publish the altitude as a simple Float64
    agl_pub_ = this->create_publisher<std_msgs::msg::Float64>("/drone/agl_distance", 10);
    
    // Also publish raw (unfiltered) for debugging
    agl_raw_pub_ = this->create_publisher<std_msgs::msg::Float64>("/drone/agl_raw", 10);
    
    RCLCPP_INFO(this->get_logger(), "AGL Parser started with odometry fallback");
    RCLCPP_INFO(this->get_logger(), "Subscribing to: /agl");
    RCLCPP_INFO(this->get_logger(), "Subscribing to: /odometry (fallback)");
    RCLCPP_INFO(this->get_logger(), "Publishing to: /drone/agl_distance (filtered)");
    RCLCPP_INFO(this->get_logger(), "Publishing to: /drone/agl_raw (unfiltered)");
    RCLCPP_INFO(this->get_logger(), "Valid range: %.2f - %.2f m", min_valid_range_, max_valid_range_);
  }

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    last_odom_ = msg;
    current_z_ = msg->pose.pose.position.z;
    
    // Auto-detect ground level when sensor gives valid low reading
    if (last_valid_range_ > 0 && last_valid_range_ < 0.5 && !ground_level_detected_) {
      ground_level_ = current_z_ - last_valid_range_;
      ground_level_detected_ = true;
      RCLCPP_INFO(this->get_logger(), 
                  "Ground level auto-detected: %.2f m (z=%.2f, agl=%.2f)", 
                  ground_level_, current_z_, last_valid_range_);
    }
  }

  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    if (msg->ranges.empty()) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                           "Received empty range data");
      publishOdomFallback();
      return;
    }
    
    // Get the single range value
    float range = msg->ranges[0];
    
    // Publish raw value for debugging
    std_msgs::msg::Float64 raw_msg;
    raw_msg.data = std::isfinite(range) ? range : -1.0;
    agl_raw_pub_->publish(raw_msg);
    
    bool use_fallback = false;
    
    // Handle invalid readings
    if (!std::isfinite(range)) {
      consecutive_invalid_++;
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                           "Invalid range reading: %s (count: %d)", 
                           std::isinf(range) ? (range > 0 ? "+inf" : "-inf") : "nan",
                           consecutive_invalid_);
      
      // If we get too many invalid readings, use odometry fallback
      if (consecutive_invalid_ > 3) {
        use_fallback = true;
      } else {
        return;
      }
    }
    // Filter out readings that are at max range
    else if (range >= max_valid_range_) {
      consecutive_invalid_++;
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                           "Range at or near maximum (%.2f m), likely invalid (count: %d)", 
                           range, consecutive_invalid_);
      
      if (consecutive_invalid_ > 3) {
        use_fallback = true;
      } else {
        return;
      }
    }
    else {
      // Valid reading received
      consecutive_invalid_ = 0;
      
      // Check if within valid bounds
      if (range < msg->range_min) {
        RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                              "Range below minimum: %.2f < %.2f, clamping", 
                              range, msg->range_min);
        range = msg->range_min;
      }
      
      // Apply median filter
      if (filter_invalid_) {
        range_history_.push_back(range);
        if (range_history_.size() > median_filter_size_) {
          range_history_.pop_front();
        }
        
        // Compute median
        if (range_history_.size() >= 3) {
          std::vector<float> sorted(range_history_.begin(), range_history_.end());
          std::sort(sorted.begin(), sorted.end());
          range = sorted[sorted.size() / 2];
        }
      }
      
      last_valid_range_ = range;
      last_valid_time_ = this->now();
      
      // Publish filtered value
      std_msgs::msg::Float64 agl_msg;
      agl_msg.data = range;
      agl_pub_->publish(agl_msg);
      
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "AGL: %.2f m (filtered, raw: %.2f)", 
                           range, msg->ranges[0]);
      return;
    }
    
    // Use odometry fallback if sensor failed
    if (use_fallback) {
      publishOdomFallback();
    }
  }
  
  void publishOdomFallback() {
    if (!use_odom_fallback_ || !last_odom_) {
      return;
    }
    
    // Check if ground level has been detected
    if (!ground_level_detected_) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                           "Ground level not yet detected, cannot use odometry fallback");
      return;
    }
    
    // Calculate AGL from odometry
    double agl_from_odom = current_z_ - ground_level_;
    
    // Clamp to reasonable values
    agl_from_odom = std::max(0.0, std::min(100.0, agl_from_odom));
    
    std_msgs::msg::Float64 agl_msg;
    agl_msg.data = agl_from_odom;
    agl_pub_->publish(agl_msg);
    
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                         "AGL: %.2f m (from odometry fallback, z=%.2f, ground=%.2f)", 
                         agl_from_odom, current_z_, ground_level_);
  }

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr agl_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr agl_raw_pub_;
  
  // Filtering parameters
  bool filter_invalid_;
  double min_valid_range_;
  double max_valid_range_;
  int median_filter_size_;
  bool use_odom_fallback_;
  
  // Odometry fallback
  nav_msgs::msg::Odometry::SharedPtr last_odom_;
  double current_z_{0.0};
  double ground_level_{0.0};
  bool ground_level_detected_{false};
  double last_valid_range_{-1.0};
  rclcpp::Time last_valid_time_{0, 0, RCL_ROS_TIME};
  int consecutive_invalid_{0};
  
  // Median filter buffer
  std::deque<float> range_history_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AglParser>());
  rclcpp::shutdown();
  return 0;
}