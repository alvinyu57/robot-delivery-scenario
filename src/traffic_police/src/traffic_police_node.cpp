#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/rgbd.hpp>
#include <opencv2/video/tracking.hpp>

#include "delivery_robot_interfaces/msg/robot_state.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "traffic_police_interfaces/msg/speed_violation.hpp"

namespace
{
constexpr char kColorTopic[] = "/delivery_robot/rgbd/image";
constexpr char kDepthTopic[] = "/delivery_robot/rgbd/depth_image";
constexpr char kCameraInfoTopic[] = "/delivery_robot/rgbd/camera_info";
constexpr char kStateTopic[] = "/delivery_robot/state";
constexpr char kViolationTopic[] = "/traffic_police/speed_violation";

rcl_interfaces::msg::ParameterDescriptor read_only_parameter(
  const std::string & description)
{
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.description = description;
  descriptor.read_only = true;
  return descriptor;
}

rclcpp::QoS reliable_topic_qos()
{
  return rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();
}

bool valid_depth(float depth)
{
  return std::isfinite(depth) && depth > 0.0F;
}
}  // namespace

class TrafficPoliceNode : public rclcpp::Node
{
public:
  TrafficPoliceNode()
  : Node("traffic_police_node")
  {
    speed_limit_ = declare_parameter<double>(
      "speed_limit", 0.5,
      read_only_parameter("Maximum permitted linear speed in metres per second"));
    if (!std::isfinite(speed_limit_) || speed_limit_ < 0.0 ||
      speed_limit_ > static_cast<double>(std::numeric_limits<float>::max()))
    {
      throw std::invalid_argument("speed_limit must be a non-negative finite float32 value");
    }
    rgbd_pair_tolerance_ = declare_parameter<double>(
      "rgbd_pair_tolerance", 0.03,
      read_only_parameter(
        "Maximum timestamp difference between color and depth images in seconds"));
    if (!std::isfinite(rgbd_pair_tolerance_) || rgbd_pair_tolerance_ < 0.0) {
      throw std::invalid_argument(
              "rgbd_pair_tolerance must be a non-negative finite value");
    }

    violation_publisher_ =
      create_publisher<traffic_police_interfaces::msg::SpeedViolation>(
      kViolationTopic, reliable_topic_qos());
    color_subscription_ = create_subscription<sensor_msgs::msg::Image>(
      kColorTopic, rclcpp::SensorDataQoS(),
      std::bind(&TrafficPoliceNode::color_callback, this, std::placeholders::_1));
    depth_subscription_ = create_subscription<sensor_msgs::msg::Image>(
      kDepthTopic, rclcpp::SensorDataQoS(),
      std::bind(&TrafficPoliceNode::depth_callback, this, std::placeholders::_1));
    camera_info_subscription_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      kCameraInfoTopic, rclcpp::SensorDataQoS(),
      std::bind(&TrafficPoliceNode::camera_info_callback, this, std::placeholders::_1));
    state_subscription_ = create_subscription<delivery_robot_interfaces::msg::RobotState>(
      kStateTopic, reliable_topic_qos(),
      std::bind(&TrafficPoliceNode::state_callback, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "Estimating speed from RGB-D topics %s and %s with a %.2f m/s limit; "
      "results publish on %s",
      kColorTopic, kDepthTopic, speed_limit_, kViolationTopic);
  }

private:
  struct RgbdFrame
  {
    rclcpp::Time stamp;
    cv::Mat gray;
    cv::Mat depth;
  };

  void color_callback(const sensor_msgs::msg::Image::ConstSharedPtr message)
  {
    latest_color_ = message;
    process_rgbd_pair();
  }

  void depth_callback(const sensor_msgs::msg::Image::ConstSharedPtr message)
  {
    latest_depth_ = message;
    process_rgbd_pair();
  }

  void camera_info_callback(
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr message)
  {
    if (message->k[0] > 0.0 && message->k[4] > 0.0) {
      camera_info_ = message;
      cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) <<
        message->k[0], 0.0, message->k[2],
        0.0, message->k[4], message->k[5],
        0.0, 0.0, 1.0);
      rgbd_odometry_ = cv::rgbd::RgbdOdometry::create(
        camera_matrix, 0.12F, 8.0F, 0.20F);
    }
  }

  void state_callback(
    const delivery_robot_interfaces::msg::RobotState::ConstSharedPtr message)
  {
    latest_state_ = *message;
  }

  cv::Mat depth_in_metres(const sensor_msgs::msg::Image & message) const
  {
    if (message.encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
      return cv_bridge::toCvCopy(message, message.encoding)->image;
    }
    if (message.encoding == sensor_msgs::image_encodings::TYPE_16UC1) {
      cv::Mat result;
      cv_bridge::toCvCopy(message, message.encoding)->image.convertTo(
        result, CV_32FC1, 0.001);
      return result;
    }
    throw cv_bridge::Exception(
            "unsupported depth encoding '" + message.encoding + "'");
  }

  void process_rgbd_pair()
  {
    if (!latest_color_ || !latest_depth_ || !camera_info_) {
      return;
    }

    const rclcpp::Time color_time(latest_color_->header.stamp);
    const rclcpp::Time depth_time(latest_depth_->header.stamp);
    if (std::abs((color_time - depth_time).seconds()) > rgbd_pair_tolerance_) {
      return;
    }
    if (last_processed_stamp_ && color_time == *last_processed_stamp_) {
      return;
    }
    last_processed_stamp_ = color_time;

    try {
      cv::Mat gray;
      const cv::Mat color = cv_bridge::toCvShare(
        latest_color_, latest_color_->encoding)->image;
      if (color.channels() == 1) {
        gray = color.clone();
      } else if (latest_color_->encoding == sensor_msgs::image_encodings::RGB8 ||
        latest_color_->encoding == sensor_msgs::image_encodings::RGBA8)
      {
        cv::cvtColor(
          color, gray,
          color.channels() == 3 ? cv::COLOR_RGB2GRAY : cv::COLOR_RGBA2GRAY);
      } else {
        cv::cvtColor(
          color, gray,
          color.channels() == 3 ? cv::COLOR_BGR2GRAY : cv::COLOR_BGRA2GRAY);
      }

      RgbdFrame current{color_time, gray, depth_in_metres(*latest_depth_)};
      if (current.gray.size() != current.depth.size()) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Ignoring RGB-D pair whose color and depth dimensions differ");
        previous_frame_.reset();
        return;
      }

      if (previous_frame_) {
        const double elapsed_seconds =
          (current.stamp - previous_frame_->stamp).seconds();
        if (elapsed_seconds > 0.0) {
          const auto speed = estimate_speed(*previous_frame_, current, elapsed_seconds);
          if (speed) {
            publish_result(*speed, current.stamp);
          } else {
            RCLCPP_WARN_THROTTLE(
              get_logger(), *get_clock(), 2000,
              "RGB-D odometry could not estimate motion from the current frame pair");
          }
        } else {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "Resetting RGB-D odometry after a non-increasing timestamp");
        }
      }
      previous_frame_ = std::move(current);
    } catch (const cv::Exception & error) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Could not process RGB-D frame: %s", error.what());
      previous_frame_.reset();
    }
  }

  std::optional<double> estimate_speed(
    const RgbdFrame & previous, const RgbdFrame & current,
    double elapsed_seconds)
  {
    cv::Mat rigid_transform;
    if (rgbd_odometry_ &&
      rgbd_odometry_->compute(
        previous.gray, previous.depth, cv::Mat(),
        current.gray, current.depth, cv::Mat(), rigid_transform) &&
      rigid_transform.rows == 4 && rigid_transform.cols == 4)
    {
      const double translation = cv::norm(
        rigid_transform(cv::Rect(3, 0, 1, 3)));
      if (std::isfinite(translation)) {
        return translation / elapsed_seconds;
      }
    }

    // Sparse 3D feature registration is a useful fallback in highly textured
    // scenes or when the dense optimizer cannot converge.
    std::vector<cv::Point2f> previous_pixels;
    cv::goodFeaturesToTrack(
      previous.gray, previous_pixels, 300, 0.01, 8.0);
    if (previous_pixels.size() < 12U) {
      return std::nullopt;
    }

    std::vector<cv::Point2f> current_pixels;
    std::vector<unsigned char> tracking_status;
    std::vector<float> tracking_error;
    cv::calcOpticalFlowPyrLK(
      previous.gray, current.gray, previous_pixels, current_pixels,
      tracking_status, tracking_error);

    const double fx = camera_info_->k[0];
    const double fy = camera_info_->k[4];
    const double cx = camera_info_->k[2];
    const double cy = camera_info_->k[5];
    std::vector<cv::Point3f> previous_points;
    std::vector<cv::Point3f> current_points;

    for (std::size_t i = 0; i < previous_pixels.size(); ++i) {
      if (!tracking_status[i] || tracking_error[i] > 30.0F) {
        continue;
      }
      const cv::Point previous_pixel(
        cvRound(previous_pixels[i].x), cvRound(previous_pixels[i].y));
      const cv::Point current_pixel(
        cvRound(current_pixels[i].x), cvRound(current_pixels[i].y));
      if (!cv::Rect(0, 0, previous.depth.cols, previous.depth.rows).contains(previous_pixel) ||
        !cv::Rect(0, 0, current.depth.cols, current.depth.rows).contains(current_pixel))
      {
        continue;
      }

      const float previous_depth = previous.depth.at<float>(previous_pixel);
      const float current_depth = current.depth.at<float>(current_pixel);
      if (!valid_depth(previous_depth) || !valid_depth(current_depth)) {
        continue;
      }
      previous_points.emplace_back(
        static_cast<float>((previous_pixels[i].x - cx) * previous_depth / fx),
        static_cast<float>((previous_pixels[i].y - cy) * previous_depth / fy),
        previous_depth);
      current_points.emplace_back(
        static_cast<float>((current_pixels[i].x - cx) * current_depth / fx),
        static_cast<float>((current_pixels[i].y - cy) * current_depth / fy),
        current_depth);
    }

    if (previous_points.size() < 12U) {
      return std::nullopt;
    }

    cv::Mat transform;
    cv::Mat inliers;
    const int inlier_count = cv::estimateAffine3D(
      previous_points, current_points, transform, inliers, 0.08, 0.99);
    if (inlier_count < 8 || transform.rows != 3 || transform.cols != 4) {
      return std::nullopt;
    }

    const double translation = std::sqrt(
      std::pow(transform.at<double>(0, 3), 2.0) +
      std::pow(transform.at<double>(1, 3), 2.0) +
      std::pow(transform.at<double>(2, 3), 2.0));
    return translation / elapsed_seconds;
  }

  void publish_result(double measured_speed, const rclcpp::Time & sample_time)
  {
    if (!latest_state_) {
      return;
    }

    traffic_police_interfaces::msg::SpeedViolation result;
    result.header = latest_state_->header;
    result.header.stamp = sample_time;
    result.robot_id = latest_state_->robot_id;
    result.pose = latest_state_->pose;
    result.measured_speed = static_cast<float>(measured_speed);
    result.speed_limit = static_cast<float>(speed_limit_);
    result.violation = measured_speed > speed_limit_;
    result.evidence_path = "";
    violation_publisher_->publish(result);

    if (result.violation) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Robot '%s' exceeded the speed limit according to RGB-D odometry: "
        "%.2f m/s > %.2f m/s",
        result.robot_id.c_str(), measured_speed, speed_limit_);
    }
  }

  double speed_limit_;
  double rgbd_pair_tolerance_;
  sensor_msgs::msg::Image::ConstSharedPtr latest_color_;
  sensor_msgs::msg::Image::ConstSharedPtr latest_depth_;
  sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info_;
  cv::Ptr<cv::rgbd::RgbdOdometry> rgbd_odometry_;
  std::optional<rclcpp::Time> last_processed_stamp_;
  std::optional<RgbdFrame> previous_frame_;
  std::optional<delivery_robot_interfaces::msg::RobotState> latest_state_;

  rclcpp::Publisher<traffic_police_interfaces::msg::SpeedViolation>::SharedPtr
    violation_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr color_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr
    camera_info_subscription_;
  rclcpp::Subscription<delivery_robot_interfaces::msg::RobotState>::SharedPtr
    state_subscription_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrafficPoliceNode>());
  rclcpp::shutdown();
  return 0;
}
