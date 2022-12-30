#ifndef _GROUND_FILTER_HPP_
#define _GROUND_FILTER_HPP_

#include <image_transport/image_transport.hpp>
#include <opencv2/core/core.hpp>
#include <pcl_ros/transforms.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <cv_bridge/cv_bridge.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

struct LidarModel
{
  int line;
  double horizon;
  double vertical;
  double resolution;
  double fov;
  double fov_up;
  double fov_down;
};

using PointType = pcl::PointXYZI;

class GroundFilter : public rclcpp::Node
{
public:
  GroundFilter();
  ~GroundFilter() = default;

  inline double radian(const double degree) { return degree * M_PI / 180.0; }
  inline double degree(const double radian) { return radian * 180.0 / M_PI; }

  void callbackSensorPoints(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  void transformPointCloud(
    pcl::PointCloud<PointType>::Ptr input_ptr, pcl::PointCloud<PointType>::Ptr & output_ptr,
    const geometry_msgs::msg::TransformStamped frame_transform);
  geometry_msgs::msg::TransformStamped getTransform(
    const std::string target_frame, const std::string source_frame);

private:
  image_transport::Publisher sensor_image_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ground_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr no_ground_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sensor_points_subscriber_;

  tf2_ros::Buffer tf2_buffer_{get_clock()};
  tf2_ros::TransformListener tf2_listener_{tf2_buffer_};

  LidarModel lidar_model_;

  int bottom_ring_;
  double angle_threshold_;
  double height_threshold_;
};

#endif