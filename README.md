# robot-delivery-scenario

An imaginary scenario of robot delivery

## Project Goal

The delivery robot moves autonomously from point A to point B and publishes its camera image and current state.

The traffic-police node observes the robot remotely, checks whether the robot exceeds the configured speed limit, and publishes a speed-violation event.

The traffic-police node is read-only toward robot motion and does not control the robot. The delivery robot cannot subscribe to the speed-violation event.

## ROS 2 Packages

The workspace contains five packages:

```text
ws/
└── src/
    ├── delivery_robot_interfaces/
    ├── delivery_robot/
    ├── traffic_police_interfaces/
    ├── traffic_police/
    └── delivery_robot_description/
```

### `delivery_robot_interfaces`

Contains custom ROS 2 message definitions.

Messages:

- `RobotState.msg`

### `traffic_police_interfaces`

Contains custom ROS 2 message definitions owned by the traffic-police domain.

Messages:

- `SpeedViolation.msg`

### `delivery_robot`

Contains `delivery_robot_node`.

Responsibilities:

- Subscribe to simulated odometry.
- Explore unknown map frontiers and navigate to one configured destination.
- Publish the robot pose and motion state.
- Change delivery state from `IDLE` to `DELIVERING`, `ARRIVED`, or `FAILED`
  based on the Nav2 action result.

Navigation is provided by:

- SLAM Toolbox for the live `map → odom` transform and occupancy map.
- Nav2 NavFn for global planning.
- Nav2 Regulated Pure Pursuit for path following and `/cmd_vel`.
- Nav2 NavigateToPose for exploration frontiers and the destination.

### `traffic_police`

Contains `traffic_police_node`.

Responsibilities:

- Subscribe to the robot camera image.
- Subscribe to the robot state.
- Determine the applicable speed limit.
- Detect speeding.
- Publish a speed-violation event.
- Optionally save an evidence image.

### `delivery_robot_description`

Contains the robot descriptions and Gazebo scenario:

- Xacro descriptions for delivery robot A and traffic police P.
- Delivery area B.
- A closed L-shaped room with a small wall near B.
- Gazebo camera sensor publishing real 800×600 RGB images.
- ROS 2 bridges for camera images and metadata, odometry, transforms, clock,
  laser scans, IMU data, and velocity commands.
- `config/gazebo_gui.config`, which loads Gazebo's lidar-ray visualization.
- `rviz/navigation.rviz`, which presents the SLAM map, robot model and TF,
  live lidar scan, and Nav2 global plan.

## Nodes and Topics

### `delivery_robot_node`

Subscribes:

| Topic | Message type | QoS | Description |
|---|---|---|---|
| `/delivery_robot/odom` | `nav_msgs/msg/Odometry` | Sensor data | Simulated robot pose and velocity |
| `/map` | `nav_msgs/msg/OccupancyGrid` | Reliable, transient local | Live SLAM map used to find reachable frontiers |

Publishes:

| Topic | Message type | QoS | Description |
|---|---|---|---|
| `/delivery_robot/state` | `delivery_robot_interfaces/msg/RobotState` | Reliable, volatile, depth 10 | Robot pose, speed, and delivery state |

Subscribes to the live SLAM occupancy map on `/map` and uses the
`/navigate_to_pose` Nav2 action. When the destination is outside known
reachable space, the node selects reachable free cells bordering unknown
space and explores them. Once the destination is mapped and connected to the
robot through known free space, it sends the final goal. Nav2 publishes
`/delivery_robot/cmd_vel`; `delivery_robot_node` does not directly control the
simulated wheels.

The Gazebo bridge, rather than `delivery_robot_node`, publishes the camera
stream on `/delivery_robot/camera/image_raw`.

### Gazebo bridge

| Topic | ROS 2 message type | Direction |
|---|---|---|
| `/clock` | `rosgraph_msgs/msg/Clock` | Gazebo → ROS 2 |
| `/delivery_robot/camera/image_raw` | `sensor_msgs/msg/Image` | Gazebo → ROS 2 |
| `/delivery_robot/camera/camera_info` | `sensor_msgs/msg/CameraInfo` | Gazebo → ROS 2 |
| `/delivery_robot/odom` | `nav_msgs/msg/Odometry` | Gazebo → ROS 2 |
| `/tf` | `tf2_msgs/msg/TFMessage` | Gazebo → ROS 2 |
| `/joint_states` | `sensor_msgs/msg/JointState` | Gazebo → ROS 2 |
| `/delivery_robot/cmd_vel` | `geometry_msgs/msg/Twist` | ROS 2 → Gazebo |
| `/scan` | `sensor_msgs/msg/LaserScan` | Gazebo → ROS 2 |
| `/imu` | `sensor_msgs/msg/Imu` | Gazebo → ROS 2 |

### Visualization

The simulation launch starts both Gazebo and RViz2 by default.

- Gazebo uses `config/gazebo_gui.config`. Its `VisualizeLidar` plugin renders
  the rays from the delivery robot's GPU lidar sensor.
- RViz2 uses `rviz/navigation.rviz` with `map` as its fixed frame. It displays
  the SLAM Toolbox occupancy map from `/map`, the delivery robot from
  `/robot_description` and `/tf`, the live scan from `/scan`, and Nav2's
  current global route from `/plan`.
- Gazebo wheel positions are bridged on `/joint_states`, allowing
  `robot_state_publisher` to keep both wheel links attached to the moving
  robot in RViz2.

The `/plan` display updates as Nav2 plans each exploration leg and the final
route to B.

### `traffic_police_node`

Subscribes:

| Topic | Message type | QoS | Description |
|---|---|---|---|
| `/delivery_robot/camera/image_raw` | `sensor_msgs/msg/Image` | Sensor data: best effort, volatile, depth 5 | Robot camera stream |
| `/delivery_robot/state` | `delivery_robot_interfaces/msg/RobotState` | Reliable, volatile, depth 10 | Robot position and speed |

Publishes:

| Topic | Message type | QoS | Description |
|---|---|---|---|
| `/traffic_police/speed_violation` | `traffic_police_interfaces/msg/SpeedViolation` | Reliable, volatile, depth 10 | Speeding event and evidence information |

## Parameters

Parameters are configured at node startup and marked read-only so accepted values cannot
silently diverge from the active publishers, subscriptions, or timer.

### `delivery_robot_node`

| Parameter | Type | Default | Validation |
|---|---|---|---|
| `robot_id` | string | `delivery_robot` | Non-empty |
| `mission_frame` | string | `map` | Non-empty |
| `destination` | double array | Required in `config/navigation.yaml` | Exactly one finite x/y/yaw triple |
| `mission_start_delay` | double | `5.0` | Non-negative |
| `occupied_threshold` | integer | `50` | From 1 to 100 |
| `frontier_min_distance` | double | `0.75` | Non-negative |
| `frontier_revisit_distance` | double | `0.75` | Non-negative |
| `obstacle_clearance` | double | `0.40` | Non-negative |

Mission coordinates are not compiled into `delivery_robot_node`. Edit the
single `destination: [x, y, yaw]` value in
`delivery_robot/config/navigation.yaml`. The node derives intermediate
frontier goals from the live occupancy map; no route waypoints are required.

### `traffic_police_node`

| Parameter | Type | Default | Validation |
|---|---|---|---|
| `speed_limit` | double | `0.5` | Non-negative finite float32 value |

## Custom Messages

### `RobotState.msg`

```text
std_msgs/Header header

string robot_id
geometry_msgs/Pose2D pose

float32 linear_speed
float32 angular_speed

uint8 IDLE=0
uint8 DELIVERING=1
uint8 ARRIVED=2
uint8 FAILED=3

uint8 delivery_state
```

### `SpeedViolation.msg`

```text
std_msgs/Header header

string robot_id
geometry_msgs/Pose2D pose

float32 measured_speed
float32 speed_limit

bool violation
string evidence_path
```

## Scripts

1. Build the Docker image:

    ```bash
    ./scripts/build-docker-image.sh
    ```

2. Run the Docker container:

    ```bash
    ./scripts/docker-it.sh
    ```

3. Build the workspace

    ```bash
    ./scripts/build-package.sh
    ./scripts/build-package.sh --docker # Build inside the Docker container
    ./scripts/build-package.sh --test # Build and run tests
    ./scripts/build-package.sh --docker --test # Docker build and tests
    ```

4. Start the Gazebo scenario from the development container:

    ```bash
    ./scripts/run-simulation.sh
    ```

    Set `GUI=false` for a headless server:

    ```bash
    GUI=false ./scripts/run-simulation.sh
    ```

`GUI=false` also keeps RViz2 disabled because its default follows the Gazebo
GUI setting. The two launch arguments can be controlled independently when
launching directly:

```bash
ros2 launch delivery_robot_description simulation.launch.py rviz:=false
ros2 launch delivery_robot_description simulation.launch.py gui:=false rviz:=true
```

The simulation publishes `/delivery_robot/camera/image_raw`,
`/delivery_robot/camera/camera_info`, `/delivery_robot/odom`, and
`/tf`. It also starts SLAM Toolbox, Nav2, and `delivery_robot_node`. The
delivery node explores toward the configured destination after mapping starts.
Robot state is copied from Gazebo odometry and changes to `ARRIVED` when Nav2
successfully reaches the destination.
