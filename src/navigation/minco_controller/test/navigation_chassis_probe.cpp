// Offline integration probe: real plugin subscriptions, public planner state,
// real MPC solve/prediction and the production /cmd_vel_mpc publishing path.
#include "minco_controller/minco_mpc_controller.hpp"
#include "minco_core/minco_planner.hpp"

#include <rclcpp/rclcpp.hpp>

#include <chrono>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  using namespace std::chrono_literals;
  rclcpp::NodeOptions plannerOptions;
  plannerOptions.parameter_overrides({
    {"use_sim_time", true},
    {"GridBased.odom_topic", "/chassis/odom"},
    {"GridBased.planner_mode", "EXPLORATION"},
    {"GridBased.rog_map.map_size", std::vector<double>{2.0, 2.0, 2.0}},
    {"GridBased.rog_map.resolution", 0.5},
    {"GridBased.rog_map.inflation_resolution", 0.5},
    {"GridBased.rog_map.decay.clear_time", 1.0},
    {"GridBased.rog_map.ros_callback.enable", false},
    {"GridBased.performance.enable", false},
    {"GridBased.performance.print_enable", false},
  });
  auto plannerNode =
    std::make_shared<nav2_util::LifecycleNode>("navigation_chassis_probe_planner", "", plannerOptions);
  auto buffer = std::make_shared<tf2_ros::Buffer>(plannerNode->get_clock());
  minco_planner::MincoPlanner planner;
  planner.configure(plannerNode, "GridBased", buffer, nullptr);

  rclcpp::NodeOptions controllerOptions;
  controllerOptions.parameter_overrides({
    {"use_sim_time", true},
    {"FollowPath.odom_topic", "/chassis/odom"},
    {"FollowPath.control_delay_compensation", 0.0},
    {"FollowPath.use_small_gyro_mode", true},
    {"FollowPath.fixed_wz", 0.2},
    {"FollowPath.performance.enable", false},
  });
  auto controllerNode = std::make_shared<rclcpp_lifecycle::LifecycleNode>(
    "navigation_chassis_probe_controller", controllerOptions);
  auto costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>("navigation_chassis_probe_costmap", true);
  costmap->set_parameter(rclcpp::Parameter("global_frame", "camera_init"));
  costmap->set_parameter(rclcpp::Parameter("robot_base_frame", "base_link_fake"));
  costmap->set_parameter(rclcpp::Parameter("plugins", std::vector<std::string>{}));
  costmap->on_configure(rclcpp_lifecycle::State());
  minco_controller::MincoMpcController controller;
  controller.configure(controllerNode, "FollowPath", buffer, costmap);

  rclcpp::NodeOptions ioOptions;
  ioOptions.parameter_overrides({{"use_sim_time", true}});
  auto io = std::make_shared<rclcpp::Node>("navigation_chassis_probe", ioOptions);
  auto posePublisher =
    io->create_publisher<geometry_msgs::msg::PoseStamped>("/chassis/test/planner_pose", 50);
  auto velocityPublisher =
    io->create_publisher<geometry_msgs::msg::TwistStamped>("/chassis/test/planner_velocity", 50);
  auto referencePublisher = io->create_publisher<ros_interfaces::msg::MpcPositionCommand>("/opt_path", 50);
  nav_msgs::msg::Odometry::ConstSharedPtr latest;
  auto subscription = io->create_subscription<nav_msgs::msg::Odometry>(
    "/chassis/odom", 50, [&](nav_msgs::msg::Odometry::ConstSharedPtr odom) {
      latest = odom;
    });
  builtin_interfaces::msg::Time fedStamp;
  uint32_t referenceId = 0;
  bool active = false;
  bool referencePending = false;
  auto timer = io->create_wall_timer(10ms, [&] {
    if (!latest) {
      return;
    }
    geometry_msgs::msg::PoseStamped pose;
    if (!planner.getRobotPose(pose) || pose.header.stamp != latest->header.stamp) {
      return;
    }
    if (!active) {
      nav_msgs::msg::Path path;
      path.header = pose.header;
      path.poses.push_back(pose);
      controller.setPlan(path);
      active = true;
    }
    if (fedStamp != pose.header.stamp) {
      fedStamp = pose.header.stamp;
      ros_interfaces::msg::MpcPositionCommand reference;
      reference.header = pose.header;
      reference.command_flag = ros_interfaces::msg::MpcPositionCommand::NORMAL_COMMAND;
      reference.cmds.resize(40);
      ++referenceId;
      for (size_t i = 0; i < reference.cmds.size(); ++i) {
        auto & command = reference.cmds[i];
        command.position = pose.pose.position;
        command.position.x += 0.05 * static_cast<double>(i);
        command.velocity.x = 1.0;
        command.yaw = planner.getCurrentYawFromOdom();
        command.trajectory_id = referenceId;
      }
      referencePublisher->publish(reference);
      referencePending = true;
      return;  // Let the real controller subscription accept the reference.
    }
    if (!referencePending) {
      return;
    }
    referencePending = false;
    posePublisher->publish(pose);
    geometry_msgs::msg::TwistStamped observed;
    observed.header = pose.header;
    const auto speed = planner.getCurrentSpeed();
    observed.twist.linear.x = speed.x();
    observed.twist.linear.y = speed.y();
    observed.twist.linear.z = speed.z();
    velocityPublisher->publish(observed);
    auto virtualPose = pose;
    virtualPose.pose.orientation = geometry_msgs::msg::Quaternion();
    virtualPose.pose.orientation.w = 1.0;
    controller.computeVelocityCommands(virtualPose, latest->twist.twist, nullptr);
  });

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(plannerNode->get_node_base_interface());
  executor.add_node(controllerNode->get_node_base_interface());
  executor.add_node(io);
  executor.spin();
  controller.cleanup();
  planner.cleanup();
  costmap->on_cleanup(rclcpp_lifecycle::State());
  rclcpp::shutdown();
  return 0;
}
