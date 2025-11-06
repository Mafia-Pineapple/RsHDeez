#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <chrono>
#include <string>
#include <vector>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <optional>
#include <cmath>

namespace fs = std::filesystem;

struct Intrinsics {
  double fx{0}, fy{0}, cx{0}, cy{0};
  bool valid() const { return fx > 0 && fy > 0; }
};

class BearDetector : public rclcpp::Node {
public:
  BearDetector() : Node("bear_detector"),
                   tf_buffer_(this->get_clock()),
                   tf_listener_(tf_buffer_) {
    // ---- Parameters ----
    thermal_topic_ = declare_parameter<std::string>("thermal_topic", "/camera/thermal/image");
    depth_topic_   = declare_parameter<std::string>("depth_topic",   "/camera/depth/image");
    camera_info_topic_ = declare_parameter<std::string>("camera_info_topic", "/camera/camera_info");
    camera_frame_  = declare_parameter<std::string>("camera_frame", "camera_thermal_optical_frame");
    map_frame_     = declare_parameter<std::string>("map_frame", "map");

    threshold_8u_  = declare_parameter<int>("threshold_8u", 220);
    threshold_16u_ = declare_parameter<int>("threshold_16u", 50000); // for 16-bit cameras
    min_hot_pixels_ = declare_parameter<int>("min_hot_pixels", 800);
    max_distance_m_ = declare_parameter<double>("max_distance", 50.0);
    detection_cooldown_s_ = declare_parameter<double>("detection_cooldown", 5.0);
    publish_goal_  = declare_parameter<bool>("publish_goal", true);
    standoff_m_    = declare_parameter<double>("standoff_distance", 2.5);
    save_dir_      = declare_parameter<std::string>("save_dir", (fs::path(getenv("HOME")) / "41068_ws/bears").string());

    fs::create_directories(save_dir_);

    // ---- Publishers ----
    det_pub_   = create_publisher<geometry_msgs::msg::PointStamped>("/bear_detection", 10);
    marker_pub_= create_publisher<visualization_msgs::msg::Marker>("/bear_marker", 10);
    status_pub_= create_publisher<std_msgs::msg::String>("/detection_status", 10);
    goal_pub_  = create_publisher<geometry_msgs::msg::PoseStamped>("/bear_goal", 10);

    // ---- Services ----
    photo_srv_ = create_service<std_srvs::srv::Trigger>(
      "/bear/save_photo",
      std::bind(&BearDetector::savePhoto, this, std::placeholders::_1, std::placeholders::_2));

    // ---- Subs ----
    thermal_sub_ = create_subscription<sensor_msgs::msg::Image>(
      thermal_topic_, 10, std::bind(&BearDetector::thermalCb, this, std::placeholders::_1));

    if (!depth_topic_.empty()) {
      depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
        depth_topic_, 10, std::bind(&BearDetector::depthCb, this, std::placeholders::_1));
    }

    cam_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      camera_info_topic_, 10, std::bind(&BearDetector::camInfoCb, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "bear_detector started. thermal=%s depth=%s map=%s",
                thermal_topic_.c_str(), depth_topic_.c_str(), map_frame_.c_str());
  }

private:
  // ---- Helpers: encoding accessors ----
  template<typename T>
  inline const T* ptrAt(const sensor_msgs::msg::Image& img, int x, int y) const {
    return reinterpret_cast<const T*>(&img.data[y * img.step + x * sizeof(T)]);
  }

  static bool is8u(const std::string& enc) {
    return enc == "mono8" || enc == "8UC1" || enc == "rgb8" || enc == "bgr8";
  }
  static bool is16u(const std::string& enc) {
    return enc == "mono16" || enc == "16UC1";
  }
  static bool isFloatDepth(const std::string& enc) {
    return enc == "32FC1";
  }

  // ---- Callbacks ----
  void camInfoCb(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
    if (!intr_.valid()) {
      intr_.fx = msg->k[0]; intr_.fy = msg->k[4];
      intr_.cx = msg->k[2]; intr_.cy = msg->k[5];
      if (intr_.valid()) {
        camera_frame_ = msg->header.frame_id;
        RCLCPP_INFO(get_logger(), "Camera intrinsics: fx=%.1f fy=%.1f cx=%.1f cy=%.1f frame=%s",
                    intr_.fx, intr_.fy, intr_.cx, intr_.cy, camera_frame_.c_str());
      }
    }
  }

  void depthCb(const sensor_msgs::msg::Image::SharedPtr msg) {
    std::lock_guard<std::mutex> lk(depth_mtx_);
    last_depth_ = *msg;
  }

  void thermalCb(const sensor_msgs::msg::Image::SharedPtr msg) {
    last_thermal_time_ = msg->header.stamp;
    last_thermal_copy_ = *msg; // for save-photo service

    if (!intr_.valid()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "No CameraInfo yet; skipping");
      return;
    }

    // Cooldown
    auto now = this->get_clock()->now();
    if ((now - last_detection_time_).seconds() < detection_cooldown_s_) {
      return;
    }

    const int W = msg->width;
    const int H = msg->height;
    const std::string enc = msg->encoding;

    // Scan hot pixels and compute centroid + bounding box
    uint64_t sum_x = 0, sum_y = 0;
    uint32_t hot = 0;
    int minx = W, miny = H, maxx = -1, maxy = -1;

    auto testPixelHot = [&](int x, int y)->bool{
      if (is8u(enc)) {
        uint8_t v;
        if (enc=="rgb8"||enc=="bgr8") {
          // average channels
          const uint8_t* p = reinterpret_cast<const uint8_t*>(&msg->data[y*msg->step + x*3]);
          v = static_cast<uint8_t>((uint16_t(p[0]) + p[1] + p[2]) / 3);
        } else {
          v = *ptrAt<uint8_t>(*msg, x, y);
        }
        return v >= threshold_8u_;
      } else if (is16u(enc)) {
        uint16_t v = *ptrAt<uint16_t>(*msg, x, y);
        return v >= threshold_16u_;
      } else {
        // Unknown encoding; try treat as 8u
        uint8_t v = *ptrAt<uint8_t>(*msg, x, y);
        return v >= threshold_8u_;
      }
    };

    for (int y=0; y<H; ++y) {
      for (int x=0; x<W; ++x) {
        if (testPixelHot(x,y)) {
          sum_x += x;
          sum_y += y;
          hot++;
          if (x < minx) minx = x;
          if (x > maxx) maxx = x;
          if (y < miny) miny = y;
          if (y > maxy) maxy = y;
        }
      }
    }

    if (hot < (uint32_t)min_hot_pixels_) {
      return; // nothing meaningful
    }

    const double cxp = double(sum_x)/double(hot);
    const double cyp = double(sum_y)/double(hot);
    const int u = static_cast<int>(std::round(cxp));
    const int v = static_cast<int>(std::round(cyp));

    // Depth lookup: use mean depth over a small window to be robust
    double Z = std::numeric_limits<double>::quiet_NaN();
    if (last_depth_.has_value()) {
      Z = depthAt(*last_depth_, u, v);
    }

    if (!std::isfinite(Z) || Z <= 0.05 || Z > max_distance_m_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "Invalid depth at centroid (u=%d v=%d): %.3f", u, v, Z);
      // Could add AGL fallback here if you want
      return;
    }

    // Back-project
    double Xc = ( (cxp - intr_.cx) * Z ) / intr_.fx;
    double Yc = ( (cyp - intr_.cy) * Z ) / intr_.fy;

    geometry_msgs::msg::PointStamped cam_pt;
    cam_pt.header = msg->header;
    cam_pt.header.frame_id = camera_frame_;
    cam_pt.point.x = Xc;
    cam_pt.point.y = Yc;
    cam_pt.point.z = Z;

    geometry_msgs::msg::PointStamped map_pt;
    try {
      auto tf = tf_buffer_.lookupTransform(map_frame_, cam_pt.header.frame_id, cam_pt.header.stamp, tf2::durationFromSec(0.2));
      tf2::doTransform(cam_pt, map_pt, tf);
    } catch (const tf2::TransformException& ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "TF error: %s", ex.what());
      return;
    }

    // Publish detection
    det_pub_->publish(map_pt);

    // Marker
    visualization_msgs::msg::Marker mk;
    mk.header.frame_id = map_frame_;
    mk.header.stamp = msg->header.stamp;
    mk.ns = "bears";
    mk.id = ++marker_id_;
    mk.type = visualization_msgs::msg::Marker::SPHERE;
    mk.action = visualization_msgs::msg::Marker::ADD;
    mk.pose.position.x = map_pt.point.x;
    mk.pose.position.y = map_pt.point.y;
    mk.pose.position.z = map_pt.point.z;
    mk.pose.orientation.w = 1.0;
    mk.scale.x = 0.8; mk.scale.y = 0.8; mk.scale.z = 0.8;
    mk.color.r = 1.0f; mk.color.g = 0.3f; mk.color.b = 0.0f; mk.color.a = 0.9f;
    mk.lifetime = rclcpp::Duration(0,0);
    marker_pub_->publish(mk);

    // Optional: publish a goal at the bear position (your motion stack decides what to do)
    if (publish_goal_) {
      geometry_msgs::msg::PoseStamped goal;
      goal.header.frame_id = map_frame_;
      goal.header.stamp = msg->header.stamp;
      goal.pose.position = mk.pose.position;
      goal.pose.orientation.w = 1.0;
      goal_pub_->publish(goal);
    }

    // Human-readable status
    std_msgs::msg::String s;
    s.data = "bear_detected at [" + std::to_string(map_pt.point.x) + "," +
             std::to_string(map_pt.point.y) + "," + std::to_string(map_pt.point.z) +
             "], hot_px=" + std::to_string(hot) + " box=(" + std::to_string(minx) + "," +
             std::to_string(miny) + ")-(" + std::to_string(maxx) + "," + std::to_string(maxy) + ")";
    status_pub_->publish(s);

    last_detection_time_ = now;
  }

  double depthAt(const sensor_msgs::msg::Image& depth, int u, int v) {
    const int W = depth.width, H = depth.height;
    if (u < 0 || v < 0 || u >= W || v >= H) return std::numeric_limits<double>::quiet_NaN();

    int win = 3; // 7x7 window
    int u0 = std::max(0, u - win), v0 = std::max(0, v - win);
    int u1 = std::min(W-1, u + win), v1 = std::min(H-1, v + win);

    double sum = 0.0; int count = 0;
    if (isFloatDepth(depth.encoding)) {
      for (int y=v0; y<=v1; ++y) {
        for (int x=u0; x<=u1; ++x) {
          float z = *ptrAt<float>(depth, x, y);
          if (std::isfinite(z) && z > 0.05) { sum += z; count++; }
        }
      }
    } else if (is16u(depth.encoding)) { // assume millimeters
      for (int y=v0; y<=v1; ++y) {
        for (int x=u0; x<=u1; ++x) {
          uint16_t zmm = *ptrAt<uint16_t>(depth, x, y);
          if (zmm > 50) { sum += double(zmm) / 1000.0; count++; }
        }
      }
    } else {
      return std::numeric_limits<double>::quiet_NaN();
    }
    if (count == 0) return std::numeric_limits<double>::quiet_NaN();
    return sum / double(count);
  }

  // ---- Service: save last thermal frame to PGM ----
  void savePhoto(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                 std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
    if (!last_thermal_copy_.has_value()) {
      res->success = false;
      res->message = "No thermal frame received yet";
      return;
    }
    const auto& img = *last_thermal_copy_;
    std::string ts = std::to_string(this->now().seconds());
    fs::path out = fs::path(save_dir_) / ("bear_" + ts + ".pgm");

    bool ok = false;
    if (is8u(img.encoding)) {
      ok = writePGM8(img, out.string());
    } else if (is16u(img.encoding)) {
      ok = writePGM16(img, out.string());
    } else {
      // fallback: dump as 8u
      ok = writePGM8(img, out.string());
    }
    res->success = ok;
    res->message = ok ? ("saved " + out.string()) : "failed to save image";
  }

  bool writePGM8(const sensor_msgs::msg::Image& img, const std::string& path) {
    std::ofstream f(path, std::ios::binary);
    if (!f) return false;
    f << "P5\n" << img.width << " " << img.height << "\n255\n";
    if (img.encoding == "rgb8" || img.encoding == "bgr8") {
      // convert to grayscale
      std::vector<uint8_t> row(img.width);
      for (uint32_t y=0; y<img.height; ++y) {
        const uint8_t* p = &img.data[y*img.step];
        for (uint32_t x=0; x<img.width; ++x) {
          row[x] = uint8_t( (uint16_t(p[0]) + p[1] + p[2]) / 3 );
          p += 3;
        }
        f.write(reinterpret_cast<const char*>(row.data()), row.size());
      }
    } else {
      for (uint32_t y=0; y<img.height; ++y) {
        const uint8_t* p = &img.data[y*img.step];
        f.write(reinterpret_cast<const char*>(p), img.width);
      }
    }
    return true;
  }

  bool writePGM16(const sensor_msgs::msg::Image& img, const std::string& path) {
    std::ofstream f(path, std::ios::binary);
    if (!f) return false;
    f << "P5\n" << img.width << " " << img.height << "\n65535\n";
    for (uint32_t y=0; y<img.height; ++y) {
      const uint8_t* p = &img.data[y*img.step];
      f.write(reinterpret_cast<const char*>(p), img.width*2);
    }
    return true;
  }

private:
  // Params
  std::string thermal_topic_, depth_topic_, camera_info_topic_;
  std::string camera_frame_, map_frame_;
  int threshold_8u_{220}, threshold_16u_{50000}, min_hot_pixels_{800};
  double max_distance_m_{50.0};
  double detection_cooldown_s_{5.0};
  bool publish_goal_{true};
  double standoff_m_{2.5};
  std::string save_dir_;

  // State
  Intrinsics intr_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Time last_detection_time_{0,0,RCL_ROS_TIME};
  int marker_id_{0};

  // Last frames
  std::mutex depth_mtx_;
  std::optional<sensor_msgs::msg::Image> last_depth_;
  std::optional<sensor_msgs::msg::Image> last_thermal_copy_;
  rclcpp::Time last_thermal_time_;

  // ROS I/O
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr thermal_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr det_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr photo_srv_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BearDetector>());
  rclcpp::shutdown();
  return 0;
}
