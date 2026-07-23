# robot-delivery-scenario
An imaginary scenario of robot delivery

## Project Goal

The delivery robot moves autonomously from point A to point B and publishes its camera image and current state.

The traffic-police node observes the robot remotely, checks whether the robot exceeds the configured speed limit, and publishes a speed-violation event.

The traffic-police node is read-only toward robot motion and does not control the robot. The delivery robot cannot subscribe to the speed-violation event.

## ROS 2 Packages

The workspace contains three packages:

```text
ws/
└── src/
    ├── delivery_robot_interfaces/
    ├── delivery_robot/
    └── traffic_police/
```

### `delivery_robot_interfaces`

Contains custom ROS 2 message definitions.

Messages:

- `RobotState.msg`
- `SpeedViolation.msg`

### `delivery_robot`

Contains `delivery_robot_node`.

Responsibilities:

- Publish the robot camera image.
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

## Nodes and Topics

### `delivery_robot_node`

Publishes:

| Topic | Message type | QoS | Description |
|---|---|---|---|
| `/delivery_robot/camera/image_raw` | `sensor_msgs/msg/Image` | Sensor data: best effort, volatile, depth 5 | Raw image from the robot camera |
| `/delivery_robot/state` | `delivery_robot_interfaces/msg/RobotState` | Reliable, volatile, depth 10 | Robot pose, speed, and delivery state |

### `traffic_police_node`

Subscribes:

| Topic | Message type | QoS | Description |
|---|---|---|---|
| `/delivery_robot/camera/image_raw` | `sensor_msgs/msg/Image` | Sensor data: best effort, volatile, depth 5 | Robot camera stream |
| `/delivery_robot/state` | `delivery_robot_interfaces/msg/RobotState` | Reliable, volatile, depth 10 | Robot position and speed |

Publishes:

| Topic | Message type | QoS | Description |
|---|---|---|---|
| `/traffic_police/speed_violation` | `delivery_robot_interfaces/msg/SpeedViolation` | Reliable, volatile, depth 10 | Speeding event and evidence information |

## Parameters

Parameters are configured at node startup and marked read-only so accepted values cannot
silently diverge from the active publishers, subscriptions, or timer.

### `delivery_robot_node`

| Parameter | Type | Default | Validation |
|---|---|---|---|
| `robot_id` | string | `delivery_robot` | Non-empty |
| `frame_id` | string | `map` | Non-empty |
| `camera_frame_id` | string | `camera_link` | Non-empty |
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
