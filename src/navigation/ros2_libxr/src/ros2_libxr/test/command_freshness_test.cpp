#include <cassert>
#include <chrono>

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <ros_interfaces/msg/behavior.hpp>

#include "ros2_libxr/timed_command.hpp"

int main()
{
  using namespace std::chrono_literals;
  using rm_serial_driver::TimedCommand;
  const auto start = std::chrono::steady_clock::time_point{} + 1s;
  TimedCommand<geometry_msgs::msg::Twist> velocity;
  geometry_msgs::msg::Twist moving;
  moving.linear.x = 1.0;
  moving.angular.z = 0.5;
  assert(velocity.value(start).linear.x == 0.0);
  velocity.update(moving, start);
  // Repeated transport reads must not extend the upstream command lifetime.
  for (int ms = 0; ms < 150; ms += 10)
    assert(velocity.value(start + std::chrono::milliseconds(ms)).linear.x == 1.0);
  assert(velocity.value(start + 150ms).linear.x == 0.0);
  assert(velocity.value(start + 150ms).angular.z == 0.0);
  velocity.update(moving, start + 200ms);
  assert(velocity.value(start + 210ms).linear.x == 1.0);

  TimedCommand<geometry_msgs::msg::WrenchStamped> force;
  geometry_msgs::msg::WrenchStamped pushing;
  pushing.wrench.force.x = 5.0;
  force.update(pushing, start);
  assert(force.value(start + 150ms).wrench.force.x == 0.0);

  TimedCommand<ros_interfaces::msg::Behavior> behavior;
  ros_interfaces::msg::Behavior escape;
  escape.tunnel_escape_active = true;
  escape.tunnel_escape_vx = 1.0;
  escape.ammo_purchase_request = 10;
  behavior.update(escape, start);
  assert(behavior.value(start + 100ms).tunnel_escape_active);
  assert(!behavior.value(start + 150ms).tunnel_escape_active);
  assert(behavior.value(start + 150ms).ammo_purchase_request == 0);
}
