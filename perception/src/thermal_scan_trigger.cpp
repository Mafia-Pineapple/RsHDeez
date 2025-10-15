#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <std_msgs/msg/bool.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <filesystem>
#include <chrono>
#include <string>
#include <mutex>

using std::placeholders::_1;

class ThermalScanTrigger : public rclcpp::Node
{
public:
  ThermalScanTrigger()
  : Node("thermal_scan_trigger")
  {
    // ---- Parameters ----
    thermal_topic_     = this->declare_parameter<std::string>("thermal_topic", "/model/scout/thermal/image");
    rgb_topic_         = this->declare_parameter<std::string>("rgb_topic",     "/model/scout/camera");
    save_dir_          = this->declare_parameter<std::string>("save_dir",      "/tmp/thermal_captures");
    hotspot_thresh_c_  = this->declare_parameter<double>("hotspot_threshold_c", 40.0); // trigger at >= 40 °C
    min_blob_area_px_  = this->declare_parameter<int>("min_blob_area_px", 150);        // ignore tiny noise
    kelvin_scale_      = this->declare_parameter<double>("kelvin_scale", 100.0);       // Gazebo often uses K*100 in mono16
    publish_preview_   = this->declare_parameter<bool>("publish_preview", true);

    std::filesystem::create_directories(save_dir_);

    // ---- Subscribers ----
    // RGB (we store most recent frame for capture)
    rgb_it_ = std::make_shared<image_transport::ImageTransport>(shared_from_this());
    rgb_sub_ = rgb_it_->subscribe(rgb_topic_, 1, std::bind(&ThermalScanTrigger::onRgb, this, _1));

    // Thermal (mono16)
    thermal_it_ = std::make_shared<image_transport::ImageTransport>(shared_from_this());
    thermal_sub_ = thermal_it_->subscribe(thermal_topic_, 1, std::bind(&ThermalScanTrigger::onThermal, this, _1));

    // ---- Publishers ----
    detection_pub_ = this->create_publisher<std_msgs::msg::Bool>("thermal/hotspot_detected", 10);
    if (publish_preview_) {
      preview_it_ = std::make_shared<image_transport::ImageTransport>(shared_from_this());
      preview_pub_ = preview_it_->advertise("thermal/preview", 1); // colorized preview (BGR8)
    }

    RCLCPP_INFO(get_logger(), "ThermalScanTrigger started");
    RCLCPP_INFO(get_logger(), "  thermal_topic: %s", thermal_topic_.c_str());
    RCLCPP_INFO(get_logger(), "  rgb_topic:     %s", rgb_topic_.c_str());
    RCLCPP_INFO(get_logger(), "  save_dir:      %s", save_dir_.c_str());
    RCLCPP_INFO(get_logger(), "  threshold:     %.2f °C | min_blob_area: %d px", hotspot_thresh_c_, min_blob_area_px_);
    RCLCPP_INFO(get_logger(), "  kelvin_scale:  %.2f", kelvin_scale_);
  }

private:
  // Latest RGB frame cache
  cv::Mat last_rgb_;
  rclcpp::Time last_rgb_stamp_;
  std::mutex rgb_mutex_;

  // Parameters / topics
  std::string thermal_topic_, rgb_topic_, save_dir_;
  double hotspot_thresh_c_, kelvin_scale_;
  int min_blob_area_px_;
  bool publish_preview_;

  // ROS objects
  image_transport::Subscriber rgb_sub_, thermal_sub_;
  std::shared_ptr<image_transport::ImageTransport> rgb_it_, thermal_it_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr detection_pub_;

  // Optional preview publisher
  std::shared_ptr<image_transport::ImageTransport> preview_it_;
  image_transport::Publisher preview_pub_;

  void onRgb(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
  {
    try {
      // Expect "rgb8" (from Gazebo rgbd camera bridge). Convert if needed.
      cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, msg->encoding);
      cv::Mat rgb;
      if (msg->encoding == sensor_msgs::image_encodings::RGB8) {
        rgb = cv_ptr->image.clone();
      } else if (msg->encoding == sensor_msgs::image_encodings::BGR8) {
        cv::cvtColor(cv_ptr->image, rgb, cv::COLOR_BGR2RGB);
      } else {
        // Convert anything else to RGB
        cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
        cv::cvtColor(cv_ptr->image, rgb, cv::COLOR_BGR2RGB);
      }

      std::lock_guard<std::mutex> lock(rgb_mutex_);
      last_rgb_ = rgb;
      last_rgb_stamp_ = msg->header.stamp;
    } catch (std::exception &e) {
      RCLCPP_WARN(get_logger(), "RGB conversion error: %s", e.what());
    }
  }

  void onThermal(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
  {
    // Expect mono16 (uint16) with Kelvin*scale
    if (msg->encoding != sensor_msgs::image_encodings::MONO16) {
      RCLCPP_WARN_THROTTLE(get_logger(), *this->get_clock(), 3000, "Thermal encoding is '%s', expected 'mono16'.", msg->encoding.c_str());
      return;
    }

    cv_bridge::CvImageConstPtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO16);
    } catch (std::exception &e) {
      RCLCPP_WARN(get_logger(), "Thermal conversion error: %s", e.what());
      return;
    }

    const cv::Mat & raw16 = cv_ptr->image; // CV_16UC1

    // Convert to Celsius float image: temp_c = raw/scale - 273.15
    cv::Mat temp_c;
    raw16.convertTo(temp_c, CV_32FC1, 1.0 / kelvin_scale_, -273.15);

    // Threshold (>= hotspot_thresh_c_)
    cv::Mat hot_mask;
    cv::threshold(temp_c, hot_mask, hotspot_thresh_c_, 255.0, cv::THRESH_BINARY);
    hot_mask.convertTo(hot_mask, CV_8UC1);

    // Morph close to fill tiny holes (optional)
    cv::morphologyEx(hot_mask, hot_mask, cv::MORPH_CLOSE, cv::Mat(), cv::Point(-1,-1), 1);

    // Find contours (blobs)
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(hot_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    bool detected = false;
    for (const auto & c : contours) {
      double area = cv::contourArea(c);
      if (area >= static_cast<double>(min_blob_area_px_)) {
        detected = true;
        break;
      }
    }

    // Publish detection flag
    std_msgs::msg::Bool flag;
    flag.data = detected;
    detection_pub_->publish(flag);

    // Optional: publish preview (colorized temp for RViz)
    if (publish_preview_ && preview_pub_.getNumSubscribers() > 0) {
      // Map temperature to 0-255 for display
      double minC = hotspot_thresh_c_ - 10.0;
      double maxC = hotspot_thresh_c_ + 20.0;
      cv::Mat norm, u8, color;
      cv::Mat clamped;
      cv::min(temp_c, maxC, clamped);
      cv::max(clamped, minC, clamped);
      clamped.convertTo(u8, CV_8UC1, 255.0 / (maxC - minC), -255.0 * minC / (maxC - minC));
      cv::applyColorMap(u8, color, cv::COLORMAP_INFERNO); // just for visualization

      // Overlay hotspots as contours
      cv::Mat overlay = color.clone();
      cv::drawContours(overlay, contours, -1, cv::Scalar(0, 255, 0), 1);
      auto out_msg = cv_bridge::CvImage(msg->header, sensor_msgs::image_encodings::BGR8, overlay).toImageMsg();
      preview_pub_.publish(out_msg);
    }

    // If detected, save latest RGB frame
    if (detected) {
      cv::Mat rgb_to_save;
      rclcpp::Time rgb_stamp;

      {
        std::lock_guard<std::mutex> lock(rgb_mutex_);
        if (!last_rgb_.empty()) {
          rgb_to_save = last_rgb_.clone();
          rgb_stamp = last_rgb_stamp_;
        }
      }

      if (!rgb_to_save.empty()) {
        // Build filename with time (sec-nsec)
        auto now = this->get_clock()->now();
        auto s = now.seconds();
        std::stringstream ss;
        ss << save_dir_ << "/capture_" << std::fixed << s << ".png";
        const std::string filepath = ss.str();

        try {
          // last_rgb_ is RGB; imwrite expects BGR by default -> convert
          cv::Mat bgr;
          cv::cvtColor(rgb_to_save, bgr, cv::COLOR_RGB2BGR);
          cv::imwrite(filepath, bgr);
          RCLCPP_INFO(get_logger(), "Hotspot detected. Saved RGB photo: %s", filepath.c_str());
        } catch (std::exception &e) {
          RCLCPP_ERROR(get_logger(), "Failed to save RGB capture: %s", e.what());
        }
      } else {
        RCLCPP_WARN(get_logger(), "Hotspot detected but no RGB frame cached yet.");
      }
    }
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ThermalScanTrigger>());
  rclcpp::shutdown();
  return 0;
}
