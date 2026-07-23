#include <chrono>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>

#include "delivery_robot_interfaces/msg/robot_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace
{
constexpr char kCameraTopic[] = "/delivery_robot/camera/image_raw";
constexpr char kStateTopic[] = "/delivery_robot/state";
}  // namespace

class DeliveryRobotNode : public rclcpp::Node
{
public:
  DeliveryRobotNode()
  : Node("delivery_robot_node")
  {
    robot_id_ = declare_parameter<std::string>("robot_id", "delivery_robot");
    frame_id_ = declare_parameter<std::string>("frame_id", "map");
    camera_frame_id_ = declare_parameter<std::string>("camera_frame_id", "camera_link");
    linear_speed_ = declare_parameter<double>("linear_speed", 1.0);
    angular_speed_ = declare_parameter<double>("angular_speed", 0.0);
    const auto publish_rate_hz = declare_parameter<double>("publish_rate_hz", 10.0);

    if (publish_rate_hz <= 0.0) {
      throw std::invalid_argument("publish_rate_hz must be greater than zero");
    }

    camera_publisher_ =
      create_publisher<sensor_msgs::msg::Image>(kCameraTopic, rclcpp::SensorDataQoS());
    state_publisher_ =
      create_publisher<delivery_robot_interfaces::msg::RobotState>(kStateTopic, 10);

    update_period_seconds_ = 1.0 / publish_rate_hz;
    const auto timer_period =
      std::chrono::duration<double>(update_period_seconds_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(timer_period),
      std::bind(&DeliveryRobotNode::publish_robot_data, this));

    RCLCPP_INFO(
      get_logger(), "Publishing camera images on %s and robot state on %s",
      kCameraTopic, kStateTopic);
  }

private:
  void publish_robot_data()
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

    sensor_msgs::msg::Image image;
    image.header.stamp = stamp;
    image.header.frame_id = camera_frame_id_;
    image.height = 1;
    image.width = 1;
    image.encoding = "rgb8";
    image.is_bigendian = false;
    image.step = 3;
    image.data = {0, 0, 0};
    camera_publisher_->publish(image);
  }

  std::string robot_id_;
  std::string frame_id_;
  std::string camera_frame_id_;
  double linear_speed_;
  double angular_speed_;
  double update_period_seconds_;
  double pose_x_{0.0};
  double pose_theta_{0.0};

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr camera_publisher_;
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
