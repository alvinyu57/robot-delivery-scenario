#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>

#include "delivery_robot_interfaces/msg/robot_state.hpp"
#include "delivery_robot_interfaces/msg/speed_violation.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace
{
constexpr char kCameraTopic[] = "/delivery_robot/camera/image_raw";
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

    violation_publisher_ =
      create_publisher<delivery_robot_interfaces::msg::SpeedViolation>(
      kViolationTopic, reliable_topic_qos());
    camera_subscription_ = create_subscription<sensor_msgs::msg::Image>(
      kCameraTopic, rclcpp::SensorDataQoS(),
      std::bind(&TrafficPoliceNode::camera_callback, this, std::placeholders::_1));
    state_subscription_ = create_subscription<delivery_robot_interfaces::msg::RobotState>(
      kStateTopic, reliable_topic_qos(),
      std::bind(&TrafficPoliceNode::state_callback, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "Monitoring %s and %s with a %.2f m/s speed limit; violations publish on %s",
      kCameraTopic, kStateTopic, speed_limit_, kViolationTopic);
  }

private:
  void camera_callback(const sensor_msgs::msg::Image::ConstSharedPtr)
  {
    has_camera_image_ = true;
  }

  void state_callback(
    const delivery_robot_interfaces::msg::RobotState::ConstSharedPtr message)
  {
    const double measured_speed = std::abs(static_cast<double>(message->linear_speed));
    if (measured_speed <= speed_limit_) {
      return;
    }

    delivery_robot_interfaces::msg::SpeedViolation violation;
    violation.header = message->header;
    violation.robot_id = message->robot_id;
    violation.pose = message->pose;
    violation.measured_speed = static_cast<float>(measured_speed);
    violation.speed_limit = static_cast<float>(speed_limit_);
    violation.violation = true;
    violation.evidence_path = "";
    violation_publisher_->publish(violation);

    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "Robot '%s' exceeded the speed limit: %.2f m/s > %.2f m/s%s",
      message->robot_id.c_str(), measured_speed, speed_limit_,
      has_camera_image_ ? " (camera data available)" : "");
  }

  double speed_limit_;
  bool has_camera_image_{false};

  rclcpp::Publisher<delivery_robot_interfaces::msg::SpeedViolation>::SharedPtr
    violation_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_subscription_;
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
