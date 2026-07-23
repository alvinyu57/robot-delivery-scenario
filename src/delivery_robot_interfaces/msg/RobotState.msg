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
