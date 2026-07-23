#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>

#include "delivery_robot_interfaces/msg/robot_state.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{
constexpr char kStateTopic[] = "/delivery_robot/state";

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

bool fits_float32(double value)
{
  return std::isfinite(value) &&
         std::abs(value) <= static_cast<double>(std::numeric_limits<float>::max());
}
}  // namespace

class DeliveryRobotNode : public rclcpp::Node
{
public:
  DeliveryRobotNode()
  : Node("delivery_robot_node")
  {
    robot_id_ = declare_parameter<std::string>(
      "robot_id", "delivery_robot", read_only_parameter("Identifier included in robot state"));
    frame_id_ = declare_parameter<std::string>(
      "frame_id", "map", read_only_parameter("Coordinate frame for robot state"));
    linear_speed_ = declare_parameter<double>(
      "linear_speed", 1.0, read_only_parameter("Simulated linear speed in metres per second"));
    angular_speed_ = declare_parameter<double>(
      "angular_speed", 0.0, read_only_parameter("Simulated angular speed in radians per second"));
    const auto publish_rate_hz = declare_parameter<double>(
      "publish_rate_hz", 10.0, read_only_parameter("Robot state publication rate in hertz"));

    validate_parameters(publish_rate_hz);

    state_publisher_ =
      create_publisher<delivery_robot_interfaces::msg::RobotState>(
      kStateTopic, reliable_topic_qos());

    update_period_seconds_ = 1.0 / publish_rate_hz;
    const auto timer_period =
      std::chrono::duration<double>(update_period_seconds_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(timer_period),
      std::bind(&DeliveryRobotNode::publish_robot_state, this));

    RCLCPP_INFO(get_logger(), "Publishing robot state on %s", kStateTopic);
  }

private:
  void validate_parameters(double publish_rate_hz) const
  {
    if (robot_id_.empty()) {
      throw std::invalid_argument("robot_id must not be empty");
    }
    if (frame_id_.empty()) {
      throw std::invalid_argument("frame_id must not be empty");
    }
    if (!fits_float32(linear_speed_)) {
      throw std::invalid_argument("linear_speed must be a finite float32 value");
    }
    if (!fits_float32(angular_speed_)) {
      throw std::invalid_argument("angular_speed must be a finite float32 value");
    }
    if (!std::isfinite(publish_rate_hz) ||
      publish_rate_hz <= 0.0 || publish_rate_hz > 1000.0)
    {
      throw std::invalid_argument("publish_rate_hz must be in the range (0, 1000]");
    }
  }

  void publish_robot_state()
  {
    const auto stamp = now();

    pose_x_ += linear_speed_ * update_period_seconds_;
    pose_theta_ += angular_speed_ * update_period_seconds_;

    delivery_robot_interfaces::msg::RobotState state;
    state.header.stamp = stamp;
    state.header.frame_id = frame_id_;
    state.robot_id = robot_id_;
    state.pose.x = pose_x_;
    state.pose.y = 0.0;
    state.pose.theta = pose_theta_;
    state.linear_speed = static_cast<float>(linear_speed_);
    state.angular_speed = static_cast<float>(angular_speed_);
    state.delivery_state = delivery_robot_interfaces::msg::RobotState::DELIVERING;
    state_publisher_->publish(state);
  }

  std::string robot_id_;
  std::string frame_id_;
  double linear_speed_;
  double angular_speed_;
  double update_period_seconds_;
  double pose_x_{0.0};
  double pose_theta_{0.0};

  rclcpp::Publisher<delivery_robot_interfaces::msg::RobotState>::SharedPtr state_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DeliveryRobotNode>());
  rclcpp::shutdown();
  return 0;
}
