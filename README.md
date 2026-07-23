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

- Publish the robot pose and motion state.
- Publish the current delivery state.

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

## Nodes and Topics

### `delivery_robot_node`

Publishes:

| Topic | Message type | QoS | Description |
|---|---|---|---|
| `/delivery_robot/state` | `delivery_robot_interfaces/msg/RobotState` | Reliable, volatile, depth 10 | Robot pose, speed, and delivery state |

The Gazebo bridge, rather than `delivery_robot_node`, publishes the camera
stream on `/delivery_robot/camera/image_raw`.

### Gazebo bridge

| Topic | ROS 2 message type | Direction |
|---|---|---|
| `/clock` | `rosgraph_msgs/msg/Clock` | Gazebo → ROS 2 |
| `/delivery_robot/camera/image_raw` | `sensor_msgs/msg/Image` | Gazebo → ROS 2 |
| `/delivery_robot/camera/camera_info` | `sensor_msgs/msg/CameraInfo` | Gazebo → ROS 2 |
| `/delivery_robot/odom` | `nav_msgs/msg/Odometry` | Gazebo → ROS 2 |
| `/delivery_robot/tf` | `tf2_msgs/msg/TFMessage` | Gazebo → ROS 2 |
| `/delivery_robot/cmd_vel` | `geometry_msgs/msg/Twist` | ROS 2 → Gazebo |
| `/scan` | `sensor_msgs/msg/LaserScan` | Gazebo → ROS 2 |
| `/imu` | `sensor_msgs/msg/Imu` | Gazebo → ROS 2 |

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
| `frame_id` | string | `map` | Non-empty |
| `linear_speed` | double | `1.0` | Finite float32 value |
| `angular_speed` | double | `0.0` | Finite float32 value |
| `publish_rate_hz` | double | `10.0` | Greater than 0 and at most 1000 |

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
    ```

4. Start the Gazebo scenario from the development container:

    ```bash
    ./scripts/run-simulation.sh
    ```

    Set `GUI=false` for a headless server:

    ```bash
    GUI=false ./scripts/run-simulation.sh
    ```

The simulation publishes `/delivery_robot/camera/image_raw`,
`/delivery_robot/camera/camera_info`, `/delivery_robot/odom`, and
`/delivery_robot/tf`. Send `geometry_msgs/msg/Twist` commands to
`/delivery_robot/cmd_vel` to move robot A.
