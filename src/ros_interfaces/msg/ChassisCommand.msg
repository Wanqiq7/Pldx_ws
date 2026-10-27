# MPC drive wrench at the chassis centre of mass.
# header.frame_id is the controller global frame; the bridge transforms to the chassis frame.
# Forces are N and torque is N*m. Only planar Fx, Fy and Mz are supported.
std_msgs/Header header
geometry_msgs/Wrench wrench
