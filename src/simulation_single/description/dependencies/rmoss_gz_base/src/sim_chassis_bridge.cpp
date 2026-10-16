#include <memory>
#include <string>

#include <geometry_msgs/msg/twist.hpp>
#include <gz/msgs/twist.pb.h>
#include <gz/transport/Node.hh>
#include <rclcpp/rclcpp.hpp>

class SimChassisBridge : public rclcpp::Node
{
public:
  SimChassisBridge()
  : Node("robot_base")
  {
    declare_parameter<std::string>("robot_name", "red_standard_robot1");
    const auto robot_name = get_parameter("robot_name").as_string();
    gz_publisher_ = gz_node_.Advertise<gz::msgs::Twist>("/" + robot_name + "/cmd_vel");
    subscription_ = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", rclcpp::QoS(10),
      [this](const geometry_msgs::msg::Twist::SharedPtr message) {
        gz::msgs::Twist command;
        command.mutable_linear()->set_x(message->linear.x);
        command.mutable_linear()->set_y(message->linear.y);
        command.mutable_linear()->set_z(message->linear.z);
        command.mutable_angular()->set_x(message->angular.x);
        command.mutable_angular()->set_y(message->angular.y);
        command.mutable_angular()->set_z(message->angular.z);
        gz_publisher_.Publish(command);
      });
  }

private:
  gz::transport::Node gz_node_;
  gz::transport::Node::Publisher gz_publisher_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimChassisBridge>());
  rclcpp::shutdown();
  return 0;
}
