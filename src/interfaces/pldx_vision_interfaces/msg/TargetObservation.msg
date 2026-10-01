# Vision -> navigation target observation.
# Topic: vision/target_observation, QoS Reliable + KeepLast(1).

# stamp: image capture time; frame_id: "vision_gimbal".
std_msgs/Header header

# Target position in header.frame_id, meters.
geometry_msgs/Point position

# Target velocity in header.frame_id, m/s.  Zero when unknown.
geometry_msgs/Vector3 velocity

# Armor/robot id, 1-based (1..8).  0 means no target.
uint8 target_id

# Detection confidence in [0, 1].
float32 confidence

# True only when position and target_id are usable.
bool valid
