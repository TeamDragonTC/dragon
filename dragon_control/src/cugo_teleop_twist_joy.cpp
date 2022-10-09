#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>

class TeleopTwistJoy : public rclcpp::Node
{
public:
  TeleopTwistJoy() : Node("teleop_twist_joy")
  {
    joy_subscriber_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "joy", 5, std::bind(&TeleopTwistJoy::callbackJoy, this, std::placeholders::_1));
    twist_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("twist", 5);
  }
  ~TeleopTwistJoy() = default;

  void callbackJoy(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    geometry_msgs::msg::Twist twist;
    twist.linear.x = msg->axes[1];
    twist.angular.z = msg->axes[3];
    twist_publisher_->publish(twist);
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscriber_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_publisher_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto teleop_twist_joy = std::make_shared<TeleopTwistJoy>();
  rclcpp::spin(teleop_twist_joy);
  rclcpp::shutdown();
  return 0;
}
