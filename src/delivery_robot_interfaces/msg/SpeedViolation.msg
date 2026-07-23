# Header and pose copied from the state sample that caused the event.
std_msgs/Header header

string robot_id
geometry_msgs/Pose2D pose

# Absolute measured linear speed and the configured limit, in metres per second.
float32 measured_speed
float32 speed_limit

# True for a confirmed violation; evidence_path is empty when no image was saved.
bool violation
string evidence_path
