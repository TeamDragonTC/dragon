#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joy.hpp>

#include <boost/asio.hpp>

class CugoTeleop : public rclcpp::Node
{
public:
  CugoTeleop() : Node("cugo_teleop_node")
  {
    wheel_base_ = this->declare_parameter("wheel_base", 0.28);
    velocity_limit_ = this->declare_parameter("velocity_limit", 0.83);

    send_str_ = "0,0";

    boost::asio::io_service io_srv;
    serial_ = std::make_shared<boost::asio::serial_port>(io_srv);

    std::string port_name = this->declare_parameter("port_name", "");
    serial_->open(port_name);
    serial_->set_option(boost::asio::serial_port_base::baud_rate(115200));
    serial_->set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
    serial_->set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));
    serial_->set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));

    twist_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 5, std::bind(&CugoTeleop::callbackTwist, this, std::placeholders::_1));

    auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(1.0 / 1));
    timer_callback_ = rclcpp::create_timer(this, get_clock(), period, std::bind(&CugoTeleop::timerCallback, this));
  }

  void callbackJoy(const sensor_msgs::msg::Joy::SharedPtr msg)
  {

  }

  void callbackTwist(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    double vel[2];
    double target_vel[2];

    vel[0] = msg->linear.x - (msg->angular.z * (wheel_base_ / 2.0));
    vel[1] = msg->linear.x + (msg->angular.z * (wheel_base_ / 2.0));

    target_vel[0] = std::min(std::max(-velocity_limit_, vel[0]), velocity_limit_);
    target_vel[1] = std::min(std::max(-velocity_limit_, vel[1]), velocity_limit_);

    target_vel[0] = target_vel[0] / velocity_limit_ * 100;
    target_vel[1] = target_vel[1] / velocity_limit_ * 100;

    std::string target_vel_str[2];

    target_vel_str[0] = std::to_string(static_cast<int>(std::round(target_vel[0])));
    target_vel_str[1] = std::to_string(static_cast<int>(std::round(target_vel[1])));

    send_str_ = target_vel_str[0] + "," + target_vel_str[1];
  }

  void timerCallback()
  {
    if(send_str_.size() > 0) {
      serial_->write_some(boost::asio::buffer(send_str_, send_str_.size()));
      std::cout << send_str_ << std::endl;
    }
  }
private:
  double wheel_base_;
  double velocity_limit_;

  std::string send_str_;

  std::shared_ptr<boost::asio::serial_port> serial_;

  rclcpp::TimerBase::SharedPtr timer_callback_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscriber_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto cugo_teleop = std::make_shared<CugoTeleop>();
  rclcpp::spin(cugo_teleop);
  rclcpp::shutdown();
  return 0;
}
