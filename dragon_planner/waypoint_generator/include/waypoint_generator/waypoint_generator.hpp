#ifndef _WAYPOINT_GENERATOR_HPP_
#define _WAYPOINT_GENERATOR_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

class WayPointGenerator : public rclcpp::Node
{
public:
  WayPointGenerator();
  ~WayPointGenerator() = default;

  void poseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

  void saveWaypoint();

private:
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_subscriber_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr waypoint_marker_publisher_;

  double threshold_;
};

#endif
