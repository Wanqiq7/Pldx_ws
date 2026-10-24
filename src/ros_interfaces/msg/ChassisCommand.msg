# Chassis command for the force-feedforward path: velocity setpoint plus the planned
# drive wrench, produced together by the MPC so the bridge forwards them atomically.
# twist and wrench_ff use the same frame as /cmd_vel_mpc (header.frame_id, the
# controller's global frame); the bridge forwards both unrotated by default.
# wrench_ff is already scaled by the controller's feedforward gain and clamped.
std_msgs/Header header
geometry_msgs/Twist twist
geometry_msgs/Wrench wrench_ff
