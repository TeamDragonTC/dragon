#include <geometry_msgs/msg/detail/polygon__struct.hpp>
#include <geometry_msgs/msg/detail/polygon_stamped__struct.hpp>
#include <pcl/common/eigen.h>
#include <pcl/impl/point_types.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/detail/point_cloud2__struct.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/polygon.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <visualization_msgs/msg/detail/marker__struct.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <pcl/point_cloud.h>
#include <pcl/common/common.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.hpp>

#ifdef USE_TF2_GEOMETRY_MSGS_DEPRECATED_HEADER
#include "tf2_eigen/tf2_eigen.h"

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include "tf2_eigen/tf2_eigen.hpp"

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif

class ObstacleSurroundStopPlanner : public rclcpp::Node
{
public:
  ObstacleSurroundStopPlanner() : Node("obstacle_surround_stop_planner")
  {
    leaf_size_ = this->declare_parameter("leaf_size", 2.0);

    std::vector<double> safety_foot_print = this->declare_parameter("foot_print", std::vector<double>(0.0, 0.0));
    for(std::size_t idx=0;idx<safety_foot_print.size();idx+=2) {
      geometry_msgs::msg::Point32 point;
      point.x = safety_foot_print[idx];
      point.y = safety_foot_print[idx+1];
      point.z = 0.0;
      safety_polygon_.points.emplace_back(point);
    }

    sensor_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("points_raw", rclcpp::SensorDataQoS().keep_last(1), std::bind(&ObstacleSurroundStopPlanner::sensorCallback, this, std::placeholders::_1));
    polygon_publisher_ = this->create_publisher<geometry_msgs::msg::PolygonStamped>("foot_print", 5);
    filtered_points_publisher_= this->create_publisher<sensor_msgs::msg::PointCloud2>("filtered_points", 5);
  }
  ~ObstacleSurroundStopPlanner() = default;

  geometry_msgs::msg::TransformStamped getTransform(const std::string target_frame, const std::string source_frame)
  {
    geometry_msgs::msg::TransformStamped frame_transform;
    try {
      frame_transform = tf2_buffer_.lookupTransform(
        target_frame, source_frame, tf2::TimePointZero, tf2::durationFromSec(0.5));
    } catch (tf2::TransformException & ex) {
      RCLCPP_ERROR(get_logger(), "%s", ex.what());
    }
    return frame_transform;
  }
  void transformPointCloud(
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr & output_ptr,
    const geometry_msgs::msg::TransformStamped frame_transform)
  {
    const Eigen::Affine3d frame_affine = tf2::transformToEigen(frame_transform);
    const Eigen::Matrix4f frame_matrix = frame_affine.matrix().cast<float>();
    pcl::transformPointCloud(*input_ptr, *output_ptr, frame_matrix);
  }


  bool searchPolygon(){}
  void downsample(const pcl::PointCloud<pcl::PointXYZ>::Ptr input_ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr &output_ptr, const double leaf_size)
  {
    pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
    voxel_grid.setInputCloud(input_ptr);
    voxel_grid.setLeafSize(leaf_size, leaf_size, leaf_size);
    voxel_grid.filter(*output_ptr);
  }

  void sensorCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::fromROSMsg(*msg, *input_cloud);
    downsample(input_cloud, filtered_cloud, leaf_size_);

    transformPointCloud(filtered_cloud, transformed_cloud, getTransform("base_link", msg->header.frame_id));

    sensor_msgs::msg::PointCloud2 filtered_points_msg;
    pcl::toROSMsg(*transformed_cloud, filtered_points_msg);
    filtered_points_msg.header.frame_id = "base_link";
    filtered_points_msg.header.stamp = msg->header.stamp;
    filtered_points_publisher_->publish(filtered_points_msg);

    geometry_msgs::msg::PolygonStamped polygon_stamped;
    polygon_stamped.header.frame_id = "base_link";
    polygon_stamped.header.stamp = msg->header.stamp;
    polygon_stamped.polygon = safety_polygon_;
    polygon_publisher_->publish(polygon_stamped);
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sensor_subscriber_;
  rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr polygon_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_points_publisher_;

  double leaf_size_;

  bool safety__check_{false};

  geometry_msgs::msg::Polygon safety_polygon_;

  tf2_ros::Buffer tf2_buffer_{get_clock()};
  tf2_ros::TransformListener tf2_listener_{tf2_buffer_};
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto obstacle_surround_stop_planner = std::make_shared<ObstacleSurroundStopPlanner>();
  rclcpp::spin(obstacle_surround_stop_planner);
  rclcpp::shutdown();
  return 0;
}
