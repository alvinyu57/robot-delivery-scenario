# Time and coordinate frame for the state sample.
std_msgs/Header header

# Stable identifier for the publishing robot.
string robot_id

# Planar robot position and heading.
geometry_msgs/Pose2D pose

# Motion in metres per second and radians per second.
float32 linear_speed
float32 angular_speed

# Delivery lifecycle states.
uint8 IDLE=0
uint8 DELIVERING=1
uint8 ARRIVED=2
uint8 FAILED=3

uint8 delivery_state
