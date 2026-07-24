#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <optional>
#include <queue>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "delivery_robot_interfaces/msg/robot_state.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2/exceptions.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"

namespace
{
constexpr char kNavigateToPoseAction[] = "/navigate_to_pose";
constexpr char kMapTopic[] = "/map";
constexpr char kOdometryTopic[] = "/delivery_robot/odom";
constexpr char kStateTopic[] = "/delivery_robot/state";
constexpr int8_t kUnknown = -1;

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

double quaternion_to_yaw(const geometry_msgs::msg::Quaternion & quaternion)
{
  const double sin_yaw = 2.0 *
    (quaternion.w * quaternion.z + quaternion.x * quaternion.y);
  const double cos_yaw = 1.0 - 2.0 *
    (quaternion.y * quaternion.y + quaternion.z * quaternion.z);
  return std::atan2(sin_yaw, cos_yaw);
}

double squared_distance(double x1, double y1, double x2, double y2)
{
  const double dx = x1 - x2;
  const double dy = y1 - y2;
  return dx * dx + dy * dy;
}
}  // namespace

class DeliveryRobotNode : public rclcpp::Node
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNavigateToPose =
    rclcpp_action::ClientGoalHandle<NavigateToPose>;

  DeliveryRobotNode()
  : Node("delivery_robot_node"),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_)
  {
    robot_id_ = declare_parameter<std::string>(
      "robot_id", "delivery_robot",
      read_only_parameter("Identifier included in robot state"));
    mission_frame_ = declare_parameter<std::string>(
      "mission_frame", "map",
      read_only_parameter("Coordinate frame used for the destination and map"));
    destination_values_ = declare_parameter<std::vector<double>>(
      "destination", std::vector<double>{},
      read_only_parameter("Required destination as one x, y, yaw triple"));
    mission_start_delay_seconds_ = declare_parameter<double>(
      "mission_start_delay", 5.0,
      read_only_parameter("Delay after first odometry before navigation starts"));
    occupied_threshold_ = declare_parameter<int>(
      "occupied_threshold", 50,
      read_only_parameter(
        "Map values at or above this percentage are not traversable"));
    frontier_min_distance_ = declare_parameter<double>(
      "frontier_min_distance", 0.75,
      read_only_parameter(
        "Minimum distance in metres from the robot to a frontier goal"));
    frontier_revisit_distance_ = declare_parameter<double>(
      "frontier_revisit_distance", 0.75,
      read_only_parameter(
        "Distance in metres used to avoid already attempted frontiers"));
    obstacle_clearance_ = declare_parameter<double>(
      "obstacle_clearance", 0.40,
      read_only_parameter(
        "Minimum map clearance in metres around reachable cells"));

    validate_parameters();

    state_publisher_ =
      create_publisher<delivery_robot_interfaces::msg::RobotState>(
      kStateTopic, reliable_topic_qos());
    odometry_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      kOdometryTopic, rclcpp::SensorDataQoS(),
      std::bind(&DeliveryRobotNode::odometry_callback, this, std::placeholders::_1));
    map_subscription_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      kMapTopic, rclcpp::QoS(1).reliable().transient_local(),
      std::bind(&DeliveryRobotNode::map_callback, this, std::placeholders::_1));
    navigation_client_ =
      rclcpp_action::create_client<NavigateToPose>(
      this, kNavigateToPoseAction);

    mission_timer_ = create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&DeliveryRobotNode::try_continue_mission, this));

    RCLCPP_INFO(
      get_logger(),
      "Waiting for Nav2 action %s, occupancy map %s, and odometry %s",
      kNavigateToPoseAction, kMapTopic, kOdometryTopic);
  }

private:
  struct Cell
  {
    int x;
    int y;
  };

  struct Frontier
  {
    geometry_msgs::msg::PoseStamped pose;
    int path_cells;
  };

  enum class GoalKind
  {
    kFrontier,
    kDestination
  };

  void validate_parameters() const
  {
    if (robot_id_.empty()) {
      throw std::invalid_argument("robot_id must not be empty");
    }
    if (mission_frame_.empty()) {
      throw std::invalid_argument("mission_frame must not be empty");
    }
    if (destination_values_.size() != 3U) {
      throw std::invalid_argument(
              "destination must contain exactly one x, y, yaw triple");
    }
    for (const double value : destination_values_) {
      if (!std::isfinite(value)) {
        throw std::invalid_argument("destination values must be finite");
      }
    }
    if (!std::isfinite(mission_start_delay_seconds_) ||
      mission_start_delay_seconds_ < 0.0)
    {
      throw std::invalid_argument("mission_start_delay must not be negative");
    }
    if (occupied_threshold_ < 1 || occupied_threshold_ > 100) {
      throw std::invalid_argument("occupied_threshold must be from 1 to 100");
    }
    if (!std::isfinite(frontier_min_distance_) ||
      frontier_min_distance_ < 0.0)
    {
      throw std::invalid_argument("frontier_min_distance must not be negative");
    }
    if (!std::isfinite(frontier_revisit_distance_) ||
      frontier_revisit_distance_ < 0.0)
    {
      throw std::invalid_argument(
              "frontier_revisit_distance must not be negative");
    }
    if (!std::isfinite(obstacle_clearance_) || obstacle_clearance_ < 0.0) {
      throw std::invalid_argument("obstacle_clearance must not be negative");
    }
  }

  void odometry_callback(const nav_msgs::msg::Odometry::ConstSharedPtr message)
  {
    latest_odometry_ = message;
    if (!has_odometry_) {
      has_odometry_ = true;
      first_odometry_time_ = now();
      RCLCPP_INFO(get_logger(), "Received first simulated odometry sample");
    }
    publish_robot_state(*message);
  }

  void map_callback(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr message)
  {
    latest_map_ = message;
  }

  void try_continue_mission()
  {
    if (mission_finished_ || goal_active_ || !has_odometry_ || !latest_map_) {
      return;
    }
    if ((now() - first_odometry_time_).seconds() <
      mission_start_delay_seconds_)
    {
      return;
    }
    if (!navigation_client_->action_server_is_ready()) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Waiting for Nav2 NavigateToPose action server");
      return;
    }

    const auto robot_pose = robot_pose_in_mission_frame();
    if (!robot_pose) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Waiting for the odom to %s transform", mission_frame_.c_str());
      return;
    }

    const auto destination = make_destination_pose();
    if (is_destination_reachable(*latest_map_, *robot_pose, destination)) {
      send_goal(destination, GoalKind::kDestination);
      return;
    }

    const auto frontier =
      find_best_frontier(*latest_map_, *robot_pose, destination);
    if (!frontier) {
      ++no_frontier_cycles_;
      if (no_frontier_cycles_ >= 10U) {
        finish_mission(
          false,
          "Destination is not reachable and no unexplored reachable frontier remains");
      }
      return;
    }

    no_frontier_cycles_ = 0U;
    send_goal(frontier->pose, GoalKind::kFrontier);
    RCLCPP_INFO(
      get_logger(),
      "Destination is outside known reachable space; exploring frontier "
      "(%.2f, %.2f), %d map cells away",
      frontier->pose.pose.position.x, frontier->pose.pose.position.y,
      frontier->path_cells);
  }

  std::optional<geometry_msgs::msg::PoseStamped>
  robot_pose_in_mission_frame()
  {
    geometry_msgs::msg::PoseStamped odometry_pose;
    odometry_pose.header = latest_odometry_->header;
    odometry_pose.pose = latest_odometry_->pose.pose;

    try {
      return tf_buffer_.transform(
        odometry_pose, mission_frame_, tf2::durationFromSec(0.2));
    } catch (const tf2::TransformException &) {
      return std::nullopt;
    }
  }

  geometry_msgs::msg::PoseStamped make_destination_pose()
  {
    return make_pose(
      destination_values_[0], destination_values_[1], destination_values_[2]);
  }

  geometry_msgs::msg::PoseStamped make_pose(double x, double y, double yaw)
  {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = now();
    pose.header.frame_id = mission_frame_;
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.orientation.z = std::sin(yaw / 2.0);
    pose.pose.orientation.w = std::cos(yaw / 2.0);
    return pose;
  }

  bool world_to_cell(
    const nav_msgs::msg::OccupancyGrid & map, double world_x, double world_y,
    Cell & cell) const
  {
    const auto & origin = map.info.origin;
    const double origin_yaw = quaternion_to_yaw(origin.orientation);
    const double dx = world_x - origin.position.x;
    const double dy = world_y - origin.position.y;
    const double local_x =
      std::cos(origin_yaw) * dx + std::sin(origin_yaw) * dy;
    const double local_y =
      -std::sin(origin_yaw) * dx + std::cos(origin_yaw) * dy;
    cell.x = static_cast<int>(std::floor(local_x / map.info.resolution));
    cell.y = static_cast<int>(std::floor(local_y / map.info.resolution));
    return cell_in_bounds(map, cell);
  }

  std::pair<double, double> cell_to_world(
    const nav_msgs::msg::OccupancyGrid & map, const Cell & cell) const
  {
    const double local_x = (static_cast<double>(cell.x) + 0.5) *
      map.info.resolution;
    const double local_y = (static_cast<double>(cell.y) + 0.5) *
      map.info.resolution;
    const double origin_yaw = quaternion_to_yaw(map.info.origin.orientation);
    return {
      map.info.origin.position.x +
      std::cos(origin_yaw) * local_x - std::sin(origin_yaw) * local_y,
      map.info.origin.position.y +
      std::sin(origin_yaw) * local_x + std::cos(origin_yaw) * local_y
    };
  }

  bool cell_in_bounds(
    const nav_msgs::msg::OccupancyGrid & map, const Cell & cell) const
  {
    return cell.x >= 0 && cell.y >= 0 &&
           cell.x < static_cast<int>(map.info.width) &&
           cell.y < static_cast<int>(map.info.height);
  }

  std::size_t cell_index(
    const nav_msgs::msg::OccupancyGrid & map, const Cell & cell) const
  {
    return static_cast<std::size_t>(cell.y) * map.info.width +
           static_cast<std::size_t>(cell.x);
  }

  bool is_free(
    const nav_msgs::msg::OccupancyGrid & map, const Cell & cell) const
  {
    if (!cell_in_bounds(map, cell)) {
      return false;
    }
    const int8_t value = map.data[cell_index(map, cell)];
    return value != kUnknown && value < occupied_threshold_;
  }

  bool has_obstacle_clearance(
    const nav_msgs::msg::OccupancyGrid & map, const Cell & cell) const
  {
    if (!is_free(map, cell)) {
      return false;
    }
    const int radius_cells = static_cast<int>(
      std::ceil(obstacle_clearance_ / map.info.resolution));
    const double radius_squared =
      obstacle_clearance_ * obstacle_clearance_;
    for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
      for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
        const double distance_squared =
          static_cast<double>(dx * dx + dy * dy) *
          map.info.resolution * map.info.resolution;
        if (distance_squared > radius_squared) {
          continue;
        }
        const Cell neighbor{cell.x + dx, cell.y + dy};
        if (!cell_in_bounds(map, neighbor)) {
          continue;
        }
        const int8_t value = map.data[cell_index(map, neighbor)];
        if (value != kUnknown && value >= occupied_threshold_) {
          return false;
        }
      }
    }
    return true;
  }

  bool is_frontier(
    const nav_msgs::msg::OccupancyGrid & map, const Cell & cell) const
  {
    if (!is_free(map, cell)) {
      return false;
    }
    for (int dy = -1; dy <= 1; ++dy) {
      for (int dx = -1; dx <= 1; ++dx) {
        if (dx == 0 && dy == 0) {
          continue;
        }
        const Cell neighbor{cell.x + dx, cell.y + dy};
        if (!cell_in_bounds(map, neighbor) ||
          map.data[cell_index(map, neighbor)] == kUnknown)
        {
          return has_obstacle_clearance(map, cell);
        }
      }
    }
    return false;
  }

  std::vector<int> reachable_distances(
    const nav_msgs::msg::OccupancyGrid & map, const Cell & start) const
  {
    std::vector<int> distances(map.data.size(), -1);
    if (!has_obstacle_clearance(map, start)) {
      return distances;
    }

    std::queue<Cell> pending;
    pending.push(start);
    distances[cell_index(map, start)] = 0;
    constexpr int kDx[] = {1, -1, 0, 0};
    constexpr int kDy[] = {0, 0, 1, -1};

    while (!pending.empty()) {
      const Cell current = pending.front();
      pending.pop();
      const int next_distance = distances[cell_index(map, current)] + 1;
      for (std::size_t direction = 0; direction < 4U; ++direction) {
        const Cell next{
          current.x + kDx[direction], current.y + kDy[direction]};
        if (!has_obstacle_clearance(map, next)) {
          continue;
        }
        const std::size_t next_index = cell_index(map, next);
        if (distances[next_index] != -1) {
          continue;
        }
        distances[next_index] = next_distance;
        pending.push(next);
      }
    }
    return distances;
  }

  bool is_destination_reachable(
    const nav_msgs::msg::OccupancyGrid & map,
    const geometry_msgs::msg::PoseStamped & robot_pose,
    const geometry_msgs::msg::PoseStamped & destination) const
  {
    Cell robot_cell{};
    Cell destination_cell{};
    if (!world_to_cell(
        map, robot_pose.pose.position.x, robot_pose.pose.position.y,
        robot_cell) ||
      !world_to_cell(
        map, destination.pose.position.x, destination.pose.position.y,
        destination_cell))
    {
      return false;
    }
    const auto distances = reachable_distances(map, robot_cell);
    return distances[cell_index(map, destination_cell)] >= 0;
  }

  std::optional<Frontier> find_best_frontier(
    const nav_msgs::msg::OccupancyGrid & map,
    const geometry_msgs::msg::PoseStamped & robot_pose,
    const geometry_msgs::msg::PoseStamped & destination)
  {
    Cell robot_cell{};
    if (!world_to_cell(
        map, robot_pose.pose.position.x, robot_pose.pose.position.y,
        robot_cell))
    {
      return std::nullopt;
    }
    const auto distances = reachable_distances(map, robot_cell);
    const double min_distance_squared =
      frontier_min_distance_ * frontier_min_distance_;
    const double revisit_distance_squared =
      frontier_revisit_distance_ * frontier_revisit_distance_;
    double best_score = std::numeric_limits<double>::infinity();
    std::optional<Frontier> best;

    for (int y = 0; y < static_cast<int>(map.info.height); ++y) {
      for (int x = 0; x < static_cast<int>(map.info.width); ++x) {
        const Cell cell{x, y};
        const int path_cells = distances[cell_index(map, cell)];
        if (path_cells < 0 || !is_frontier(map, cell)) {
          continue;
        }
        const auto [world_x, world_y] = cell_to_world(map, cell);
        if (squared_distance(
            world_x, world_y, robot_pose.pose.position.x,
            robot_pose.pose.position.y) < min_distance_squared)
        {
          continue;
        }
        const bool already_attempted = std::any_of(
          attempted_frontiers_.begin(), attempted_frontiers_.end(),
          [world_x, world_y, revisit_distance_squared](const auto & point) {
            return squared_distance(
              world_x, world_y, point.first, point.second) <
                   revisit_distance_squared;
          });
        if (already_attempted) {
          continue;
        }

        const double destination_distance = std::sqrt(
          squared_distance(
            world_x, world_y, destination.pose.position.x,
            destination.pose.position.y));
        const double path_distance =
          static_cast<double>(path_cells) * map.info.resolution;
        const double score = destination_distance + 0.15 * path_distance;
        if (score >= best_score) {
          continue;
        }

        const double yaw = std::atan2(
          destination.pose.position.y - world_y,
          destination.pose.position.x - world_x);
        best_score = score;
        best = Frontier{make_pose(world_x, world_y, yaw), path_cells};
      }
    }
    return best;
  }

  void send_goal(
    const geometry_msgs::msg::PoseStamped & pose, GoalKind kind)
  {
    NavigateToPose::Goal goal;
    goal.pose = pose;

    rclcpp_action::Client<NavigateToPose>::SendGoalOptions options;
    options.goal_response_callback =
      std::bind(
      &DeliveryRobotNode::goal_response_callback, this,
      std::placeholders::_1);
    options.result_callback =
      std::bind(
      &DeliveryRobotNode::result_callback, this,
      std::placeholders::_1);

    active_goal_kind_ = kind;
    active_goal_pose_ = pose;
    goal_active_ = true;
    navigation_client_->async_send_goal(goal, options);
  }

  void goal_response_callback(
    const GoalHandleNavigateToPose::SharedPtr & goal_handle)
  {
    if (!goal_handle) {
      handle_navigation_failure("Nav2 rejected the navigation goal");
      return;
    }

    delivery_state_ =
      delivery_robot_interfaces::msg::RobotState::DELIVERING;
    publish_latest_robot_state();
    RCLCPP_INFO(
      get_logger(), "Nav2 accepted the %s goal",
      active_goal_kind_ == GoalKind::kDestination ?
      "destination" : "exploration");
  }

  void result_callback(
    const GoalHandleNavigateToPose::WrappedResult & result)
  {
    goal_active_ = false;
    if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
      handle_navigation_failure("Nav2 could not reach the navigation goal");
      return;
    }

    if (active_goal_kind_ == GoalKind::kDestination) {
      finish_mission(true, "Reached the configured destination");
      return;
    }

    attempted_frontiers_.emplace_back(
      active_goal_pose_.pose.position.x, active_goal_pose_.pose.position.y);
    RCLCPP_INFO(
      get_logger(), "Reached exploration frontier; checking the expanded map");
  }

  void handle_navigation_failure(const std::string & reason)
  {
    goal_active_ = false;
    if (active_goal_kind_ == GoalKind::kDestination) {
      finish_mission(false, reason);
      return;
    }

    attempted_frontiers_.emplace_back(
      active_goal_pose_.pose.position.x, active_goal_pose_.pose.position.y);
    RCLCPP_WARN(
      get_logger(), "%s at frontier (%.2f, %.2f); trying another frontier",
      reason.c_str(), active_goal_pose_.pose.position.x,
      active_goal_pose_.pose.position.y);
  }

  void finish_mission(bool succeeded, const std::string & message)
  {
    mission_finished_ = true;
    delivery_state_ = succeeded ?
      delivery_robot_interfaces::msg::RobotState::ARRIVED :
      delivery_robot_interfaces::msg::RobotState::FAILED;
    publish_latest_robot_state();
    if (succeeded) {
      RCLCPP_INFO(get_logger(), "%s; delivery state is ARRIVED", message.c_str());
    } else {
      RCLCPP_ERROR(get_logger(), "%s; delivery state is FAILED", message.c_str());
    }
  }

  void publish_latest_robot_state()
  {
    if (latest_odometry_) {
      publish_robot_state(*latest_odometry_);
    }
  }

  void publish_robot_state(const nav_msgs::msg::Odometry & odometry)
  {
    const auto & position = odometry.pose.pose.position;
    const auto & velocity = odometry.twist.twist;

    delivery_robot_interfaces::msg::RobotState state;
    state.header = odometry.header;
    state.robot_id = robot_id_;
    state.pose.x = position.x;
    state.pose.y = position.y;
    state.pose.theta = quaternion_to_yaw(odometry.pose.pose.orientation);
    state.linear_speed = static_cast<float>(
      std::hypot(velocity.linear.x, velocity.linear.y));
    state.angular_speed = static_cast<float>(velocity.angular.z);
    state.delivery_state = delivery_state_;
    state_publisher_->publish(state);
  }

  std::string robot_id_;
  std::string mission_frame_;
  std::vector<double> destination_values_;
  double mission_start_delay_seconds_;
  int occupied_threshold_;
  double frontier_min_distance_;
  double frontier_revisit_distance_;
  double obstacle_clearance_;
  bool has_odometry_{false};
  bool goal_active_{false};
  bool mission_finished_{false};
  uint32_t no_frontier_cycles_{0};
  uint8_t delivery_state_{
    delivery_robot_interfaces::msg::RobotState::IDLE};
  GoalKind active_goal_kind_{GoalKind::kFrontier};
  geometry_msgs::msg::PoseStamped active_goal_pose_;
  std::vector<std::pair<double, double>> attempted_frontiers_;
  rclcpp::Time first_odometry_time_;
  nav_msgs::msg::Odometry::ConstSharedPtr latest_odometry_;
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr latest_map_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Publisher<delivery_robot_interfaces::msg::RobotState>::SharedPtr
    state_publisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr
    odometry_subscription_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr
    map_subscription_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr navigation_client_;
  rclcpp::TimerBase::SharedPtr mission_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DeliveryRobotNode>());
  rclcpp::shutdown();
  return 0;
}
