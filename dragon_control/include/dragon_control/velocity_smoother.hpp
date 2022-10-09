#ifndef _VELOCITY_SMOOTHER_
#define _VELOCITY_SMOOTHER_

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>

class VelocitySmoother : public rclcpp::Node
{
public:
  VelocitySmoother();
  ~VelocitySmoother() = default;

private:
  void twistCallback(const geometry_msgs::msg::Twist msg);
  void timerCallback();
  geometry_msgs::msg::Twist filteredTwist(const geometry_msgs::msg::Twist twist);

private:
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr raw_cmd_vel_subscriber_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr filtered_cmd_vel_publisher_;

  geometry_msgs::msg::Twist target_cmd_vel_;
  geometry_msgs::msg::Twist last_cmd_vel_;

  rclcpp::Time last_time_stamp_;

  rclcpp::TimerBase::SharedPtr timer_;

  double velocity_gain_;
  double angular_gain_;

  double maximum_limit_vel_;
  double minimum_limit_vel_;
};

#endif
