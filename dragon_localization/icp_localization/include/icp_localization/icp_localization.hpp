#ifndef _NDT_LOCALIZATION_
#define _NDT_LOCALIZATION_

#include <geometry_msgs/msg/detail/pose_with_covariance_stamped__struct.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/detail/point_cloud2__struct.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/float32.hpp>

#include <tf2/transform_datatypes.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

#ifdef USE_TF2_GEOMETRY_MSGS_DEPRECATED_HEADER
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif

#include <pcl/filters/voxel_grid.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/gicp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.hpp>

#include <fast_gicp/gicp/fast_gicp.hpp>

class ICPLocalization : public rclcpp::Node
{
  using PointType = pcl::PointXYZ;

public:
  ICPLocalization();
  ~ICPLocalization() = default;

private:
  void mapCallback(const sensor_msgs::msg::PointCloud2& map);
  void pointsCallback(const sensor_msgs::msg::PointCloud2& points);
  void initialPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped& initialpose);

  void
  downsample(const pcl::PointCloud<PointType>::Ptr& input_cloud_ptr, pcl::PointCloud<PointType>::Ptr& output_cloud_ptr);
  void crop(
    const pcl::PointCloud<PointType>::Ptr& input_cloud_ptr, pcl::PointCloud<PointType>::Ptr output_cloud_ptr,
    const double min_range, const double max_range);

  void
  publishTF(const std::string frame_id, const std::string child_frame_id, const geometry_msgs::msg::PoseStamped pose);

private:
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr map_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr points_subscriber_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initialpose_subscriber_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr icp_align_cloud_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr icp_pose_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr icp_pose_with_covariance_publisher_;

  // icp
  std::shared_ptr<pcl::Registration<PointType, PointType>> registration_;

  geometry_msgs::msg::Pose initial_pose_;

  tf2_ros::Buffer tf_buffer_{ get_clock() };
  std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_;

  // config for icp omp
  double max_correspondence_distance_;
  double euclidean_fitness_epsilon_;
  double ransac_outlier_rejection_threshold_;
  double transformation_epsilon_;
  int max_iteration_;
  std::string map_frame_id_;
  std::string base_frame_id_;

  double downsample_leaf_size_;

  double min_range_;
  double max_range_;

  bool localization_ready_{ false };
};

#endif
