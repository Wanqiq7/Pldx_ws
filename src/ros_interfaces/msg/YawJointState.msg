# Big Yaw joint state sampled from ONE motor feedback frame (contract frozen by Pldx_ws#6).
#
# header.stamp    Measurement time of the motor frame on the ROS/LIO clock. It is the MCU
#                 sample time mapped through a clock-offset estimate, never the receive time.
# header.frame_id Reference frame the angle is measured against: the physical chassis ("base_link").
# child_frame_id  Frame of the rotating end that carries the lidar ("lidar_yaw_mount").
#
# Both values are about the same axis, measured on the chassis side.
# angle           rad, rotating end relative to the chassis, in [-pi, pi). It is NOT the lidar
#                 heading in the world. Zero is the motor calibration zero (yaw_zero);
#                 positive is counter-clockwise seen from above after the motor `reverse` setting.
# velocity        rad/s, d(angle)/dt of the same sample and sign. It is NOT the lidar or chassis
#                 angular rate in the world.
std_msgs/Header header
string child_frame_id
float64 angle
float64 velocity

# Diagnostics. mcu_stamp_us is the 48-bit LibXR packet timestamp (MCU microseconds, wraps at 2^48).
# receive_stamp is when the host decoded the packet; it is not the measurement time.
uint64 mcu_stamp_us
builtin_interfaces/Time receive_stamp
