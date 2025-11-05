#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class BearDetector : public rclcpp::Node {
public:
    BearDetector() : Node("bear_detector") {
        this->declare_parameter("min_area", 3000);           // Min pixels
        this->declare_parameter("max_distance", 15.0);       // Max meters
        this->declare_parameter("detection_cooldown", 5.0);  // Seconds between detections
        
        min_area_ = this->get_parameter("min_area").as_int();
        max_distance_ = this->get_parameter("max_distance").as_double();
        detection_cooldown_ = this->get_parameter("detection_cooldown").as_double();
        
        // TF
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        
        // Subscribers
        rgb_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/image", 10,
            std::bind(&BearDetector::imageCallback, this, std::placeholders::_1));
        
        depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/depth/image", 10,
            std::bind(&BearDetector::depthCallback, this, std::placeholders::_1));
        
        camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            "/camera/camera_info", 10,
            std::bind(&BearDetector::cameraInfoCallback, this, std::placeholders::_1));
        
        // Publishers
        detection_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>(
            "/bear_detection", 10);
        
        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
            "/bear_marker", 10);
        
        status_pub_ = this->create_publisher<std_msgs::msg::String>(
            "/detection_status", 10);
        
        RCLCPP_INFO(this->get_logger(), "Bear Detector initialized (color-based)");
    }

private:
    void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
        if (!has_camera_info_) {
            fx_ = msg->k[0];
            fy_ = msg->k[4];
            cx_ = msg->k[2];
            cy_ = msg->k[5];
            has_camera_info_ = true;
            RCLCPP_INFO(this->get_logger(), "Camera calibrated");
        }
    }
    
    void depthCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        try {
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, 
                sensor_msgs::image_encodings::TYPE_32FC1);
            depth_image_ = cv_ptr->image;
            has_depth_ = true;
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Depth error: %s", e.what());
        }
    }
    
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        if (!has_camera_info_ || !has_depth_) return;
        
        // Cooldown check
        auto now = this->now();
        if ((now - last_detection_time_).seconds() < detection_cooldown_) {
            return;
        }
        
        try {
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, 
                sensor_msgs::image_encodings::BGR8);
            cv::Mat rgb = cv_ptr->image;
            
            // ===== WHITE BEAR DETECTION =====
            // Convert to grayscale
            cv::Mat gray;
            cv::cvtColor(rgb, gray, cv::COLOR_BGR2GRAY);
            
            // Threshold for white (brightness > 220)
            cv::Mat mask;
            cv::threshold(gray, mask, 220, 255, cv::THRESH_BINARY);
            
            // Clean up noise
            cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(7,7));
            cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);
            cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);
            
            // Optional: Show detection mask for debugging
            // cv::imshow("White Bear Detection", mask);
            // cv::waitKey(1);
            
            // Find contours
            std::vector<std::vector<cv::Point>> contours;
            cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
            
            // Find largest white region
            double max_area = 0;
            int best_idx = -1;
            for (size_t i = 0; i < contours.size(); ++i) {
                double area = cv::contourArea(contours[i]);
                if (area > max_area && area > min_area_) {
                    max_area = area;
                    best_idx = i;
                }
            }
            
            if (best_idx >= 0) {
                processBearDetection(contours[best_idx], msg->header.stamp);
                
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                   "White region detected! Area: %.0f pixels", max_area);
            }
            
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Image error: %s", e.what());
        }
    }
    
    void processBearDetection(const std::vector<cv::Point>& contour, 
                             const rclcpp::Time& timestamp) {
        // Get bounding box center
        cv::Rect bbox = cv::boundingRect(contour);
        int cx = bbox.x + bbox.width / 2;
        int cy = bbox.y + bbox.height / 2;
        
        // Get depth
        if (cx >= depth_image_.cols || cy >= depth_image_.rows) return;
        float depth = depth_image_.at<float>(cy, cx);
        
        if (!std::isfinite(depth) || depth <= 0.1 || depth > max_distance_) {
            return;
        }
        
        // Convert to 3D point in camera frame
        geometry_msgs::msg::PointStamped point_camera;
        point_camera.header.stamp = timestamp;
        point_camera.header.frame_id = "camera_depth_optical_frame";
        point_camera.point.x = (cx - cx_) * depth / fx_;
        point_camera.point.y = (cy - cy_) * depth / fy_;
        point_camera.point.z = depth;
        
        // Transform to map frame
        try {
            geometry_msgs::msg::PointStamped point_map;
            tf_buffer_->transform(point_camera, point_map, "map", 
                                 tf2::durationFromSec(0.5));
            
            // Publish detection
            detection_pub_->publish(point_map);
            
            // Visualize
            publishMarker(point_map);
            
            // Status
            std_msgs::msg::String status;
            status.data = "BEAR_DETECTED";
            status_pub_->publish(status);
            
            last_detection_time_ = this->now();
            
            RCLCPP_INFO(this->get_logger(), 
                       "BEAR DETECTED at (%.2f, %.2f, %.2f) - distance: %.2fm",
                       point_map.point.x, point_map.point.y, point_map.point.z, depth);
            
        } catch (tf2::TransformException& ex) {
            RCLCPP_WARN(this->get_logger(), "TF error: %s", ex.what());
        }
    }
    
    void publishMarker(const geometry_msgs::msg::PointStamped& point) {
        visualization_msgs::msg::Marker marker;
        marker.header = point.header;
        marker.ns = "bears";
        marker.id = detection_count_++;
        marker.type = visualization_msgs::msg::Marker::CUBE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        
        marker.pose.position = point.point;
        marker.pose.orientation.w = 1.0;
        
        marker.scale.x = 0.8;
        marker.scale.y = 0.8;
        marker.scale.z = 1.5;  // Bear height
        
        marker.color.r = 0.6;
        marker.color.g = 0.3;
        marker.color.b = 0.0;
        marker.color.a = 0.8;
        
        marker.lifetime = rclcpp::Duration::from_seconds(0);  // Permanent
        
        marker_pub_->publish(marker);
    }
    
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rgb_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
    
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr detection_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
    
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    
    cv::Mat depth_image_;
    bool has_camera_info_ = false;
    bool has_depth_ = false;
    double fx_, fy_, cx_, cy_;
    
    int min_area_;
    double max_distance_;
    double detection_cooldown_;
    rclcpp::Time last_detection_time_{0, 0, RCL_ROS_TIME};
    int detection_count_ = 0;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BearDetector>());
    rclcpp::shutdown();
    return 0;
}