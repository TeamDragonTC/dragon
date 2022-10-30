#include <waypoint_generator/waypoint_generator.hpp>

WayPointGenerator::WayPointGenerator() : Node("waypoint_generator")
{
  pose_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("pose_with_covariance", 5, std::bind(&WayPointGenerator::poseCallback, this, std::placeholders::_1));

  waypoint_marker_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("waypoint_marker", 5);
}

void WayPointGenerator::poseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
}
