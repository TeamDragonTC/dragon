#include <geometry_msgs/msg/detail/quaternion__struct.hpp>
#include <geometry_msgs/msg/detail/twist_with_covariance__struct.hpp>
#include <geometry_msgs/msg/detail/twist_with_covariance_stamped__struct.hpp>
#include <geometry_msgs/msg/detail/vector3__struct.hpp>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/quaternion.hpp>

#include <tf2/LinearMath/Quaternion.h>

#ifdef USE_TF2_GEOMETRY_MSGS_DEPRECATED_HEADER
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif

class OdomToTwist : public rclcpp::Node
{
public:
  OdomToTwist() : Node("odom2twist")
  {
    odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>("odom", 5, std::bind(&OdomToTwist::callbackOdom, this, std::placeholders::_1));

    twist_publisher_ = this->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>("twist", 5);
  }
  ~OdomToTwist() = default;

  void callbackOdom(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    static nav_msgs::msg::Odometry prev_odom = *msg;

    geometry_msgs::msg::TwistWithCovarianceStamped twist_stamped;

    const double dt = (rclcpp::Time(msg->header.stamp) - rclcpp::Time(prev_odom.header.stamp)).seconds();
    if(dt < 1e-05) {
      twist_stamped.header = msg->header;
      twist_stamped.header.frame_id = "base_link";
      twist_publisher_->publish(twist_stamped);
    }

    const double dx = msg->pose.pose.position.x - prev_odom.pose.pose.position.x;
    const double dy = msg->pose.pose.position.y - prev_odom.pose.pose.position.y;
    const double dz = msg->pose.pose.position.z - prev_odom.pose.pose.position.z;

    geometry_msgs::msg::Vector3 rpy = convertToRPY(msg->pose.pose.orientation);
    geometry_msgs::msg::Vector3 prev_rpy = convertToRPY(prev_odom.pose.pose.orientation);

    twist_stamped.header = msg->header;
    twist_stamped.header.frame_id = "base_link";
    twist_stamped.twist.twist.linear.x = std::sqrt(std::pow(dx, 2) + std::pow(dy, 2) + std::pow(dz, 2)) / dt;
    twist_stamped.twist.twist.linear.y = 0.0;//dy / dt;
    twist_stamped.twist.twist.linear.z = 0.0;//dz / dt;
    twist_stamped.twist.twist.angular.x = calcDiffForRadian(rpy.x, prev_rpy.x) / dt;
    twist_stamped.twist.twist.angular.y = calcDiffForRadian(rpy.y, prev_rpy.y) / dt;
    twist_stamped.twist.twist.angular.z = calcDiffForRadian(rpy.z, prev_rpy.z) / dt;

    twist_publisher_->publish(twist_stamped);

    prev_odom = *msg;
  }

  geometry_msgs::msg::Vector3 convertToRPY(const geometry_msgs::msg::Quaternion quaternion)
  {
    tf2::Quaternion tf2_quat(quaternion.x, quaternion.y, quaternion.z, quaternion.w);
    tf2::Matrix3x3 mat(tf2_quat);
    geometry_msgs::msg::Vector3 rpy;
    mat.getRPY(rpy.x, rpy.y, rpy.z);
    return rpy;
  }

  double calcDiffForRadian(const double lhs_rad, const double rhs_rad)
  {
    double diff_rad = lhs_rad - rhs_rad;
    if (M_PI <= diff_rad)
      diff_rad -= 2 * M_PI;
    else if (diff_rad < -M_PI)
      diff_rad += 2 * M_PI;
    return diff_rad;
  }

private:
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
  rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr twist_publisher_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto odom2twist = std::make_shared<OdomToTwist>();
  rclcpp::spin(odom2twist);
  rclcpp::shutdown();
  return 0;
}
