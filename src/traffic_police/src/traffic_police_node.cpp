#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>

#include "delivery_robot_interfaces/msg/robot_state.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "traffic_police_interfaces/msg/speed_violation.hpp"

namespace
{
constexpr char kLeftImageTopic[] = "/delivery_robot/stereo/left/image_raw";
constexpr char kRightImageTopic[] = "/delivery_robot/stereo/right/image_raw";
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
    stereo_pair_tolerance_ = declare_parameter<double>(
      "stereo_pair_tolerance", 0.05,
      read_only_parameter(
        "Maximum timestamp difference between left and right images in seconds"));
    if (!std::isfinite(stereo_pair_tolerance_) || stereo_pair_tolerance_ < 0.0) {
      throw std::invalid_argument(
              "stereo_pair_tolerance must be a non-negative finite value");
    }

    violation_publisher_ =
      create_publisher<traffic_police_interfaces::msg::SpeedViolation>(
      kViolationTopic, reliable_topic_qos());
    left_image_subscription_ = create_subscription<sensor_msgs::msg::Image>(
      kLeftImageTopic, rclcpp::SensorDataQoS(),
      std::bind(&TrafficPoliceNode::left_image_callback, this, std::placeholders::_1));
    right_image_subscription_ = create_subscription<sensor_msgs::msg::Image>(
      kRightImageTopic, rclcpp::SensorDataQoS(),
      std::bind(&TrafficPoliceNode::right_image_callback, this, std::placeholders::_1));
    state_subscription_ = create_subscription<delivery_robot_interfaces::msg::RobotState>(
      kStateTopic, reliable_topic_qos(),
      std::bind(&TrafficPoliceNode::state_callback, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "Monitoring stereo images %s and %s plus poses on %s with a %.2f m/s "
      "speed limit; compliance results publish on %s",
      kLeftImageTopic, kRightImageTopic, kStateTopic, speed_limit_, kViolationTopic);
  }

private:
  void left_image_callback(const sensor_msgs::msg::Image::ConstSharedPtr message)
  {
    latest_left_image_stamp_ = rclcpp::Time(message->header.stamp);
  }

  void right_image_callback(const sensor_msgs::msg::Image::ConstSharedPtr message)
  {
    latest_right_image_stamp_ = rclcpp::Time(message->header.stamp);
  }

  bool has_synchronized_stereo_pair() const
  {
    if (!latest_left_image_stamp_ || !latest_right_image_stamp_) {
      return false;
    }
    return std::abs(
      (*latest_left_image_stamp_ - *latest_right_image_stamp_).seconds()) <=
           stereo_pair_tolerance_;
  }

  void state_callback(
    const delivery_robot_interfaces::msg::RobotState::ConstSharedPtr message)
  {
    const rclcpp::Time sample_time(message->header.stamp);
    if (!previous_state_) {
      previous_state_ = *message;
      return;
    }

    const rclcpp::Time previous_time(previous_state_->header.stamp);
    const double elapsed_seconds = (sample_time - previous_time).seconds();
    if (elapsed_seconds <= 0.0 || message->robot_id != previous_state_->robot_id ||
      message->header.frame_id != previous_state_->header.frame_id)
    {
      previous_state_ = *message;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Resetting speed calculation after a non-increasing timestamp, robot ID, "
        "or coordinate-frame change");
      return;
    }

    const double distance = std::hypot(
      message->pose.x - previous_state_->pose.x,
      message->pose.y - previous_state_->pose.y);
    const double measured_speed = distance / elapsed_seconds;
    previous_state_ = *message;

    traffic_police_interfaces::msg::SpeedViolation result;
    result.header = message->header;
    result.robot_id = message->robot_id;
    result.pose = message->pose;
    result.measured_speed = static_cast<float>(measured_speed);
    result.speed_limit = static_cast<float>(speed_limit_);
    result.violation = measured_speed > speed_limit_;
    result.evidence_path = "";
    violation_publisher_->publish(result);

    if (result.violation) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Robot '%s' exceeded the speed limit: %.2f m/s > %.2f m/s%s",
        message->robot_id.c_str(), measured_speed, speed_limit_,
        has_synchronized_stereo_pair() ? " (stereo data available)" : "");
    }
  }

  double speed_limit_;
  double stereo_pair_tolerance_;
  std::optional<rclcpp::Time> latest_left_image_stamp_;
  std::optional<rclcpp::Time> latest_right_image_stamp_;
  std::optional<delivery_robot_interfaces::msg::RobotState> previous_state_;

  rclcpp::Publisher<traffic_police_interfaces::msg::SpeedViolation>::SharedPtr
    violation_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr
    left_image_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr
    right_image_subscription_;
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
