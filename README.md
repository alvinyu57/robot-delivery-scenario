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
    ├── delivery_interfaces/
    ├── delivery_robot/
    └── traffic_police/
```

### `delivery_interfaces`

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

| Topic | Message type | Description |
|---|---|---|
| `/delivery_robot/camera/image_raw` | `sensor_msgs/msg/Image` | Raw image from the robot camera |
| `/delivery_robot/state` | `delivery_interfaces/msg/RobotState` | Robot pose, speed, and delivery state |

### `traffic_police_node`

Subscribes:

| Topic | Message type | Description |
|---|---|---|
| `/delivery_robot/camera/image_raw` | `sensor_msgs/msg/Image` | Robot camera stream |
| `/delivery_robot/state` | `delivery_interfaces/msg/RobotState` | Robot position and speed |

Publishes:

| Topic | Message type | Description |
|---|---|---|
| `/traffic_police/speed_violation` | `delivery_interfaces/msg/SpeedViolation` | Speeding event and evidence information |

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

