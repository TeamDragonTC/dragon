#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <sensor_msgs/msg/joy.hpp>

class TeleopTwistJoy : public rclcpp::Node
{
public:
  TeleopTwistJoy() : Node("teleop_twist_joy")
  {
    joy_subscriber_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "joy", 5, std::bind(&TeleopTwistJoy::callbackJoy, this, std::placeholders::_1));
    twist_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("twist", 5);
    cmd_publisher_ = this->create_publisher<std_msgs::msg::Bool>("cmd", 5);
  }
  ~TeleopTwistJoy() = default;

  void callbackJoy(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    static auto prev_joy_msg = msg;

    geometry_msgs::msg::Twist twist;
    twist.linear.x = msg->axes[1];
    twist.angular.z = msg->axes[3];
    twist_publisher_->publish(twist);

    if(prev_joy_msg->buttons[1] == 0 and msg->buttons[1] == 1) {
      std_msgs::msg::Bool cmd;
      cmd.data = false;
      cmd_publisher_->publish(cmd);
    }

    prev_joy_msg = msg;
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscriber_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr cmd_publisher_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto teleop_twist_joy = std::make_shared<TeleopTwistJoy>();
  rclcpp::spin(teleop_twist_joy);
  rclcpp::shutdown();
  return 0;
}
