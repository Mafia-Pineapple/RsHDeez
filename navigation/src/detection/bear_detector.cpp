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
        this->declare_parameter("min_area", 3000);
        this->declare_parameter("max_area", 200000);
        this->declare_parameter("min_aspect_ratio", 0.3);
        this->declare_parameter("max_aspect_ratio", 3.0);
        this->declare_parameter("max_white_percentage", 50.0);
        this->declare_parameter("max_distance", 15.0);
        this->declare_parameter("detection_cooldown", 5.0);

        min_area_ = this->get_parameter("min_area").as_int();
        max_area_ = this->get_parameter("max_area").as_int();
        min_aspect_ratio_ = this->get_parameter("min_aspect_ratio").as_double();
        max_aspect_ratio_ = this->get_parameter("max_aspect_ratio").as_double();
        max_white_percentage_ = this->get_parameter("max_white_percentage").as_double();
        max_distance_ = this->get_parameter("max_distance").as_double();
        detection_cooldown_ = this->get_parameter("detection_cooldown").as_double();

        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        rgb_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/model/scout/thermal/image", 10,
            std::bind(&BearDetector::imageCallback, this, std::placeholders::_1));

        depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/depth/image", 10,
            std::bind(&BearDetector::depthCallback, this, std::placeholders::_1));

        camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            "/camera/camera_info", 10,
            std::bind(&BearDetector::cameraInfoCallback, this, std::placeholders::_1));

        detection_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>(
            "/bear_detection", 10);
        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
            "/bear_marker", 10);
        status_pub_ = this->create_publisher<std_msgs::msg::String>(
            "/detection_status", 10);

        goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/bear_goal", 10);

        RCLCPP_INFO(this->get_logger(), "Bear Detector initialized");
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

    auto now = this->now();
    if ((now - last_detection_time_).seconds() < detection_cooldown_) {
        return;
    }

    try {
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg,
            sensor_msgs::image_encodings::BGR8);
        cv::Mat rgb = cv_ptr->image;

        // Convert to grayscale
        cv::Mat gray;
        cv::cvtColor(rgb, gray, cv::COLOR_BGR2GRAY);

        // Threshold for white
        cv::Mat mask;
        cv::threshold(gray, mask, 220, 255, cv::THRESH_BINARY);

        // Morphology to clean up noise
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(7,7));
        cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);
        cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);

        int white_pixels = cv::countNonZero(mask);
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
            "White pixels: %d / %d", white_pixels, mask.rows*mask.cols);

        // Find contours
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        double max_area = 0;
        int best_idx = -1;
        for (size_t i = 0; i < contours.size(); ++i) {
            double area = cv::contourArea(contours[i]);
            if (area < min_area_ || area > max_area_) continue;

            cv::Rect bbox = cv::boundingRect(contours[i]);
            double aspect_ratio = static_cast<double>(bbox.width) / bbox.height;
            if (aspect_ratio < min_aspect_ratio_ || aspect_ratio > max_aspect_ratio_)
                continue;

            if (area > max_area) {
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
        cv::Rect bbox = cv::boundingRect(contour);
        int cx = bbox.x + bbox.width / 2;
        int cy = bbox.y + bbox.height / 2;

        if (cx >= depth_image_.cols || cy >= depth_image_.rows) return;
        float depth = depth_image_.at<float>(cy, cx);
        if (!std::isfinite(depth) || depth <= 0.1 || depth > max_distance_) return;

        geometry_msgs::msg::PointStamped point_camera;
        point_camera.header.stamp = timestamp;
        point_camera.header.frame_id = "camera_depth_optical_frame";
        point_camera.point.x = (cx - cx_) * depth / fx_;
        point_camera.point.y = (cy - cy_) * depth / fy_;
        point_camera.point.z = depth;

        try {
            geometry_msgs::msg::PointStamped point_map;
            tf_buffer_->transform(point_camera, point_map, "map",
                                  tf2::durationFromSec(0.5));

            detection_pub_->publish(point_map);
            publishMarker(point_map);

            std_msgs::msg::String status;
            status.data = "BEAR_DETECTED";
            status_pub_->publish(status);

            geometry_msgs::msg::PoseStamped goal;
            goal.header = point_map.header;
            goal.pose.position = point_map.point;
            goal.pose.orientation.w = 1.0;
            goal_pub_->publish(goal);

            last_detection_time_ = this->now();

            RCLCPP_INFO(this->get_logger(),
                "BEAR DETECTED at (%.2f, %.2f, %.2f) - distance: %.2fm",
                point_map.point.x, point_map.point.y, point_map.point.z, depth);
            RCLCPP_INFO(this->get_logger(), "Published bear goal for navigation.");

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
        marker.scale.z = 1.5;
        marker.color.r = 0.6;
        marker.color.g = 0.3;
        marker.color.b = 0.0;
        marker.color.a = 0.8;
        marker.lifetime = rclcpp::Duration::from_seconds(0);
        marker_pub_->publish(marker);
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rgb_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;

    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr detection_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    cv::Mat depth_image_;
    bool has_camera_info_ = false;
    bool has_depth_ = false;
    double fx_, fy_, cx_, cy_;
    int min_area_;
    int max_area_;
    double min_aspect_ratio_;
    double max_aspect_ratio_;
    double max_white_percentage_;
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
