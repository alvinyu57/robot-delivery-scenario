# RGB-D sample time with the coordinate frame and latest pose from robot state.
std_msgs/Header header

string robot_id
geometry_msgs/Pose2D pose

# Absolute measured linear speed and the configured limit, in metres per second.
float32 measured_speed
float32 speed_limit

# True when measured_speed exceeds speed_limit.
# evidence_path is empty when no RGB-D evidence was saved.
bool violation
string evidence_path
