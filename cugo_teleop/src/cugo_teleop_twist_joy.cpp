#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist.hpp>

class TeleopTwistJoy : public rclcpp::Node
{
public:
  TeleopTwistJoy() : Node("teleop_twist_joy")
  {
  }
  ~TeleopTwistJoy() = default;

  void callbackJoy(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscriber_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_publisher_;
};

int main(int argc, char** argv)
{

}
