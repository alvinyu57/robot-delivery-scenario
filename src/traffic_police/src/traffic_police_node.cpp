#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/rgbd.hpp>
#include <opencv2/video/tracking.hpp>

#include "delivery_robot_interfaces/msg/robot_state.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/synchronizer.h"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "traffic_police/motion_estimator.hpp"
#include "traffic_police_interfaces/msg/speed_violation.hpp"

namespace
{
constexpr char kColorTopic[] = "/delivery_robot/rgbd/image";
constexpr char kDepthTopic[] = "/delivery_robot/rgbd/depth_image";
constexpr char kCameraInfoTopic[] = "/delivery_robot/rgbd/camera_info";
constexpr char kStateTopic[] = "/delivery_robot/state";
constexpr char kViolationTopic[] = "/traffic_police/speed_violation";
constexpr uint32_t kRgbdSynchronizerQueueSize = 10U;

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
  using Image = sensor_msgs::msg::Image;
  using RgbdSyncPolicy =
    message_filters::sync_policies::ApproximateTime<Image, Image>;
  using RgbdSynchronizer = message_filters::Synchronizer<RgbdSyncPolicy>;

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
    const auto camera_extrinsics = declare_parameter<std::vector<double>>(
      "camera_extrinsics",
      std::vector<double>{
      0.36, 0.0, 0.37,
      -1.5707963267948966, 0.0, -1.5707963267948966},
      read_only_parameter(
        "Camera optical-frame pose in base_footprint as x, y, z, roll, pitch, yaw"));
    if (camera_extrinsics.size() != 6U) {
      throw std::invalid_argument(
              "camera_extrinsics must contain x, y, z, roll, pitch, yaw");
    }
    for (const double value : camera_extrinsics) {
      if (!std::isfinite(value)) {
        throw std::invalid_argument("camera_extrinsics values must be finite");
      }
    }
    const double maximum_linear_speed = declare_parameter<double>(
      "maximum_plausible_linear_speed", 1.0,
      read_only_parameter(
        "Reject RGB-D estimates above this base linear speed in metres per second"));
    const double maximum_angular_speed = declare_parameter<double>(
      "maximum_plausible_angular_speed", 2.0,
      read_only_parameter(
        "Reject RGB-D estimates above this base angular speed in radians per second"));
    const double maximum_sample_interval = declare_parameter<double>(
      "maximum_sample_interval", 0.5,
      read_only_parameter(
        "Reject motion estimates spanning more than this many seconds"));
    if (!std::isfinite(maximum_linear_speed) ||
      maximum_linear_speed <= speed_limit_)
    {
      throw std::invalid_argument(
              "maximum_plausible_linear_speed must be finite and greater than speed_limit");
    }
    if (!std::isfinite(maximum_angular_speed) || maximum_angular_speed <= 0.0) {
      throw std::invalid_argument(
              "maximum_plausible_angular_speed must be positive and finite");
    }
    if (!std::isfinite(maximum_sample_interval) || maximum_sample_interval <= 0.0) {
      throw std::invalid_argument(
              "maximum_sample_interval must be positive and finite");
    }

    traffic_police::MotionEstimatorConfig estimator_config;
    estimator_config.base_from_camera = traffic_police::make_rigid_transform(
      camera_extrinsics[0], camera_extrinsics[1], camera_extrinsics[2],
      camera_extrinsics[3], camera_extrinsics[4], camera_extrinsics[5]);
    estimator_config.maximum_linear_speed = maximum_linear_speed;
    estimator_config.maximum_angular_speed = maximum_angular_speed;
    estimator_config.maximum_sample_interval = maximum_sample_interval;
    motion_estimator_ =
      std::make_unique<traffic_police::MotionEstimator>(estimator_config);

    violation_publisher_ =
      create_publisher<traffic_police_interfaces::msg::SpeedViolation>(
      kViolationTopic, reliable_topic_qos());
    color_subscription_.subscribe(
      this, kColorTopic, rmw_qos_profile_sensor_data);
    depth_subscription_.subscribe(
      this, kDepthTopic, rmw_qos_profile_sensor_data);
    RgbdSyncPolicy synchronization_policy(kRgbdSynchronizerQueueSize);
    synchronization_policy.setMaxIntervalDuration(
      rclcpp::Duration::from_seconds(rgbd_pair_tolerance_));
    rgbd_synchronizer_ = std::make_shared<RgbdSynchronizer>(
      synchronization_policy);
    rgbd_synchronizer_->connectInput(
      color_subscription_, depth_subscription_);
    rgbd_synchronizer_->registerCallback(
      std::bind(
        &TrafficPoliceNode::rgbd_callback, this,
        std::placeholders::_1, std::placeholders::_2));
    camera_info_subscription_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      kCameraInfoTopic, rclcpp::SensorDataQoS(),
      std::bind(&TrafficPoliceNode::camera_info_callback, this, std::placeholders::_1));
    state_subscription_ = create_subscription<delivery_robot_interfaces::msg::RobotState>(
      kStateTopic, reliable_topic_qos(),
      std::bind(&TrafficPoliceNode::state_callback, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "Estimating base speed from synchronized RGB-D topics %s and %s with "
      "a %.2f m/s limit; results publish on %s",
      kColorTopic, kDepthTopic, speed_limit_, kViolationTopic);
  }

private:
  struct RgbdFrame
  {
    rclcpp::Time stamp;
    cv::Mat gray;
    cv::Mat depth;
  };

  void rgbd_callback(
    const Image::ConstSharedPtr & color,
    const Image::ConstSharedPtr & depth)
  {
    process_rgbd_pair(color, depth);
  }

  void camera_info_callback(
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr message)
  {
    if (!std::isfinite(message->k[0]) || message->k[0] <= 0.0 ||
      !std::isfinite(message->k[2]) ||
      !std::isfinite(message->k[4]) || message->k[4] <= 0.0 ||
      !std::isfinite(message->k[5]) ||
      message->width == 0U || message->height == 0U)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Ignoring camera calibration with invalid intrinsics");
      camera_info_.reset();
      rgbd_odometry_.release();
      previous_frame_.reset();
      return;
    }

    const bool calibration_changed =
      !camera_info_ ||
      camera_info_->width != message->width ||
      camera_info_->height != message->height ||
      camera_info_->header.frame_id != message->header.frame_id ||
      camera_info_->k != message->k;
    if (!calibration_changed) {
      return;
    }

    try {
      cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) <<
        message->k[0], 0.0, message->k[2],
        0.0, message->k[4], message->k[5],
        0.0, 0.0, 1.0);
      auto rgbd_odometry = cv::rgbd::RgbdOdometry::create(
        camera_matrix, 0.12F, 8.0F, 0.20F);
      camera_info_ = message;
      rgbd_odometry_ = std::move(rgbd_odometry);
      previous_frame_.reset();
    } catch (const cv::Exception & error) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Could not configure RGB-D odometry: %s", error.what());
      camera_info_.reset();
      rgbd_odometry_.release();
      previous_frame_.reset();
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

  void process_rgbd_pair(
    const Image::ConstSharedPtr & color_message,
    const Image::ConstSharedPtr & depth_message)
  {
    if (!camera_info_) {
      return;
    }

    const rclcpp::Time color_time(color_message->header.stamp);
    const rclcpp::Time depth_time(depth_message->header.stamp);
    if (std::abs((color_time - depth_time).seconds()) > rgbd_pair_tolerance_) {
      return;
    }
    if (!color_message->header.frame_id.empty() &&
      !depth_message->header.frame_id.empty() &&
      color_message->header.frame_id != depth_message->header.frame_id)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Ignoring RGB-D pair whose frame IDs differ");
      previous_frame_.reset();
      return;
    }
    if ((color_message->width != camera_info_->width) ||
      (color_message->height != camera_info_->height) ||
      (depth_message->width != camera_info_->width) ||
      (depth_message->height != camera_info_->height) ||
      (!camera_info_->header.frame_id.empty() &&
      ((!color_message->header.frame_id.empty() &&
      color_message->header.frame_id != camera_info_->header.frame_id) ||
      (!depth_message->header.frame_id.empty() &&
      depth_message->header.frame_id != camera_info_->header.frame_id))))
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Ignoring RGB-D pair that does not match camera calibration");
      previous_frame_.reset();
      return;
    }

    try {
      cv::Mat gray;
      const cv::Mat color = cv_bridge::toCvShare(
        color_message, color_message->encoding)->image;
      if (color.channels() == 1) {
        gray = color.clone();
      } else if (color_message->encoding == sensor_msgs::image_encodings::RGB8 ||
        color_message->encoding == sensor_msgs::image_encodings::RGBA8)
      {
        cv::cvtColor(
          color, gray,
          color.channels() == 3 ? cv::COLOR_RGB2GRAY : cv::COLOR_RGBA2GRAY);
      } else {
        cv::cvtColor(
          color, gray,
          color.channels() == 3 ? cv::COLOR_BGR2GRAY : cv::COLOR_BGRA2GRAY);
      }

      RgbdFrame current{color_time, gray, depth_in_metres(*depth_message)};
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
          const auto motion = estimate_motion(
            *previous_frame_, current, elapsed_seconds);
          if (motion) {
            publish_result(motion->linear_speed, current.stamp);
          } else {
            RCLCPP_WARN_THROTTLE(
              get_logger(), *get_clock(), 2000,
              "RGB-D motion was unavailable or failed plausibility checks");
          }
        } else {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "Resetting RGB-D odometry after a non-increasing timestamp");
        }
      }
      previous_frame_ = std::move(current);
    } catch (const cv_bridge::Exception & error) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Could not convert RGB-D frame: %s", error.what());
      previous_frame_.reset();
    } catch (const cv::Exception & error) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Could not process RGB-D frame: %s", error.what());
      previous_frame_.reset();
    } catch (const std::exception & error) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Unexpected RGB-D processing failure: %s", error.what());
      previous_frame_.reset();
    }
  }

  std::optional<traffic_police::MotionEstimate> estimate_motion(
    const RgbdFrame & previous, const RgbdFrame & current,
    double elapsed_seconds)
  {
    std::optional<traffic_police::MotionEstimate> dense_motion;
    cv::Mat rigid_transform;
    if (rgbd_odometry_ &&
      rgbd_odometry_->compute(
        previous.gray, previous.depth, cv::Mat(),
        current.gray, current.depth, cv::Mat(), rigid_transform) &&
      rigid_transform.rows == 4 && rigid_transform.cols == 4)
    {
      dense_motion = motion_estimator_->estimate_from_camera_transform(
        rigid_transform, elapsed_seconds);
    }

    // Prefer the sparse path because it supplies explicit RANSAC inliers and
    // residual validation. Dense odometry remains a bounded fallback for
    // scenes that do not contain enough trackable features.
    std::vector<cv::Point2f> previous_pixels;
    cv::goodFeaturesToTrack(
      previous.gray, previous_pixels, 300, 0.01, 8.0);
    if (previous_pixels.size() < 12U) {
      return dense_motion;
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
      return dense_motion;
    }
    const auto sparse_motion = motion_estimator_->estimate_from_correspondences(
      previous_points, current_points, elapsed_seconds);
    return sparse_motion ? sparse_motion : dense_motion;
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
  sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info_;
  cv::Ptr<cv::rgbd::RgbdOdometry> rgbd_odometry_;
  std::unique_ptr<traffic_police::MotionEstimator> motion_estimator_;
  std::optional<RgbdFrame> previous_frame_;
  std::optional<delivery_robot_interfaces::msg::RobotState> latest_state_;

  rclcpp::Publisher<traffic_police_interfaces::msg::SpeedViolation>::SharedPtr
    violation_publisher_;
  message_filters::Subscriber<Image> color_subscription_;
  message_filters::Subscriber<Image> depth_subscription_;
  std::shared_ptr<RgbdSynchronizer> rgbd_synchronizer_;
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
