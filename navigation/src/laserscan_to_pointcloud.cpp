#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <laser_geometry/laser_geometry.hpp>

class LaserScanToPointCloud : public rclcpp::Node {
public:
  LaserScanToPointCloud() : Node("laserscan_to_pointcloud") {
    sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10,
        std::bind(&LaserScanToPointCloud::scanCallback, this, std::placeholders::_1));
    
    pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/scan/points", 10);
    
    RCLCPP_INFO(this->get_logger(), "LaserScan to PointCloud converter started");
  }

private:
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
    sensor_msgs::msg::PointCloud2 cloud;
    projector_.projectLaser(*scan, cloud);
    cloud.header = scan->header;
    pub_->publish(cloud);
  }

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
  laser_geometry::LaserProjection projector_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LaserScanToPointCloud>());
  rclcpp::shutdown();
  return 0;
}