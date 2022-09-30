#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

class TwistPathEstimator : public rclcpp::Node
{
public:
  TwistPathEstimator() : Node("twist_path_estimator")
  {
    path_marker_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("estimate_path_marker", 5);
    twist_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 5, std::bind(&TwistPathEstimator::twistCallback, this, std::placeholders::_1));
  }
  ~TwistPathEstimator() = default;

  void twistCallback(const geometry_msgs::msg::Twist msg)
  {
    double velocity = msg.linear.x;
    double omega = msg.angular.z;
    if(std::fabs(msg.angular.z) < 1e-06)
      omega = 1e-06;

    const double radius = velocity / omega;

    // create visualization marker
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.ns = "estimate_path";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.lifetime = rclcpp::Duration::from_seconds(0.0);
    marker.pose.position.x = 0.0;
    marker.pose.position.y = 0.0;
    marker.pose.position.z = 0.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.05;
    marker.scale.y = 0.0;
    marker.scale.z = 0.0;
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;
    marker.frame_locked = true;
  }
  geometry_msgs::msg::Point transform2D(const geometry_msgs::msg::Point point , const geometry_msgs::msg::Pose pose)
  {
  }

private:
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_subscriber_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr estimate_path_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr path_marker_publisher_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto twist_path_estimator = std::make_shared<TwistPathEstimator>();
  rclcpp::spin(twist_path_estimator);
  rclcpp::shutdown();
  return 0;
}
