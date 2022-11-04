#ifndef _WAYPOINT_GENERATOR_HPP_
#define _WAYPOINT_GENERATOR_HPP_
#include <cstdint>
#include <fstream>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <std_msgs/msg/detail/color_rgba__struct.hpp>
#include <visualization_msgs/msg/marker.hpp>

class WayPointGenerator : public rclcpp::Node
{
public:
  WayPointGenerator();
  ~WayPointGenerator() = default;

  void poseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

  double getDist(const geometry_msgs::msg::Pose pose_1, const geometry_msgs::msg::Pose pose_2);
  double getVelocity(const geometry_msgs::msg::Pose pose_1, const geometry_msgs::msg::Pose pose_2, const double dt);

  geometry_msgs::msg::Vector3 createScale(const double x, const double y, const double z);
  std_msgs::msg::ColorRGBA createColor(const double a, const double r, const double g, const double b);
  visualization_msgs::msg::Marker createMarker(
    const geometry_msgs::msg::Pose pose, const int32_t type, const int id, const std::string text,
    geometry_msgs::msg::Vector3 scale, std_msgs::msg::ColorRGBA color);

private:
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_subscriber_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr waypoint_marker_publisher_;

  int id_{ 0 };
  int way_point_id_{ 0 };
  double threshold_;
  std::string way_point_csv_path_;
};

#endif
