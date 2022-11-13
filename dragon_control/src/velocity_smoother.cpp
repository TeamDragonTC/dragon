#include <dragon_control/velocity_smoother.hpp>

VelocitySmoother::VelocitySmoother() : Node("velocity_smoother")
{
  velocity_gain_ = this->declare_parameter<double>("velocity_gain");
  angular_gain_ = this->declare_parameter<double>("angular_gain");
  maximum_limit_vel_ = this->declare_parameter<double>("maximum_limit_vel");
  minimum_limit_vel_ = this->declare_parameter<double>("minimum_limit_vel");
  maximum_limit_omega_ = this->declare_parameter<double>("maximum_limit_omega");
  minimum_limit_omega_ = this->declare_parameter<double>("minimum_limit_omega");

  raw_cmd_vel_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "raw_cmd_vel", 1, std::bind(&VelocitySmoother::twistCallback, this, std::placeholders::_1));
  filtered_cmd_vel_publisher_ =
    this->create_publisher<geometry_msgs::msg::Twist>("filtered_cmd_vel", 1);

  last_time_stamp_ = rclcpp::Clock().now();

  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(static_cast<int>(1000 * 0.01)),
    std::bind(&VelocitySmoother::timerCallback, this));
}

void VelocitySmoother::twistCallback(const geometry_msgs::msg::Twist msg)
{
  // check velocity limit
  target_cmd_vel_.linear.x = msg.linear.x > 0.0 ? std::min(msg.linear.x, maximum_limit_vel_)
                                                : std::max(msg.linear.x, minimum_limit_vel_);
  target_cmd_vel_.angular.z = msg.angular.z > 0.0 ? std::min(msg.angular.z, maximum_limit_omega_)
                                                  : std::max(msg.angular.z, minimum_limit_omega_);
}

void VelocitySmoother::timerCallback()
{
  geometry_msgs::msg::Twist cmd_vel;

  rclcpp::Time current_time_stamp = rclcpp::Clock().now();
  const double dt = (current_time_stamp - last_time_stamp_).seconds();

  const double delta_linear_vel =
    velocity_gain_ * (target_cmd_vel_.linear.x - last_cmd_vel_.linear.x);
  const double delta_angular_vel =
    angular_gain_ * (target_cmd_vel_.angular.z - last_cmd_vel_.angular.z);

  cmd_vel.linear.x = delta_linear_vel * dt + last_cmd_vel_.linear.x;
  cmd_vel.angular.z = delta_angular_vel * dt + last_cmd_vel_.angular.z;

  filtered_cmd_vel_publisher_->publish(cmd_vel);

  last_cmd_vel_ = cmd_vel;
  last_time_stamp_ = current_time_stamp;
}
