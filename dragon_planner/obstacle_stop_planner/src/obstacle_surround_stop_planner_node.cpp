#include <pcl_ros/transforms.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/polygon.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <pcl/common/common.h>
#include <pcl/common/eigen.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

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

    x_min_ = this->declare_parameter("x_min", 0.0);
    x_max_ = this->declare_parameter("x_max", 0.0);
    y_min_ = this->declare_parameter("y_min", 0.0);
    y_max_ = this->declare_parameter("y_max", 0.0);
    z_min_ = this->declare_parameter("z_min", 0.0);
    z_max_ = this->declare_parameter("z_max", 0.0);

    use_debug_ = this->declare_parameter("use_debug", false);

    std::vector<double> safety_foot_print =
      this->declare_parameter("foot_print", std::vector<double>(0.0, 0.0));
    for (std::size_t idx = 0; idx < safety_foot_print.size(); idx += 2) {
      geometry_msgs::msg::Point32 point;
      point.x = safety_foot_print[idx];
      point.y = safety_foot_print[idx + 1];
      point.z = 0.0;
      safety_polygon_.points.emplace_back(point);
    }

    sensor_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "points_raw", rclcpp::SensorDataQoS().keep_last(1),
      std::bind(&ObstacleSurroundStopPlanner::sensorCallback, this, std::placeholders::_1));
    twist_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel_in", 5,
      std::bind(&ObstacleSurroundStopPlanner::twistCallback, this, std::placeholders::_1));
    polygon_publisher_ =
      this->create_publisher<geometry_msgs::msg::PolygonStamped>("foot_print", 5);
    filtered_points_publisher_ =
      this->create_publisher<sensor_msgs::msg::PointCloud2>("filtered_points", 5);
    twist_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel_out", 5);
    if (use_debug_)
      marker_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("marker", 5);
  }
  ~ObstacleSurroundStopPlanner() = default;

  geometry_msgs::msg::TransformStamped getTransform(
    const std::string target_frame, const std::string source_frame)
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

  bool searchPolygon(const pcl::PointXYZ point)
  {
    if (safety_polygon_.points.size() < 3) return false;

    double sum_angle = 0;
    for (std::size_t idx = 0; idx < safety_polygon_.points.size(); ++idx) {
      const double vec_1_x = safety_polygon_.points.at(idx).x - point.x;
      const double vec_1_y = safety_polygon_.points.at(idx).y - point.y;
      const double vec_2_x =
        safety_polygon_.points.at((idx + 1) % safety_polygon_.points.size()).x - point.x;
      const double vec_2_y =
        safety_polygon_.points.at((idx + 1) % safety_polygon_.points.size()).y - point.y;

      const double vec_1 = vec_1_x * vec_2_x + vec_1_y * vec_2_y;
      const double vec_2 = vec_1_y * vec_2_x - vec_1_x * vec_2_y;

      const double angle = std::atan2(vec_2, vec_1);
      sum_angle += angle;
    }

    return (2 * M_PI - std::fabs(sum_angle) < 0.001) ? true : false;
  }
  void downsample(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr input_ptr,
    pcl::PointCloud<pcl::PointXYZ>::Ptr & output_ptr, const double leaf_size)
  {
    pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
    voxel_grid.setInputCloud(input_ptr);
    voxel_grid.setLeafSize(leaf_size, leaf_size, leaf_size);
    voxel_grid.filter(*output_ptr);
  }

  void crop(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr input_ptr,
    pcl::PointCloud<pcl::PointXYZ>::Ptr & output_ptr)
  {
    for (auto point : input_ptr->points) {
      if (
        x_min_ < point.x && point.x < x_max_ && y_min_ < point.y && point.y < y_max_ &&
        z_min_ < point.z && point.z < z_max_) {
        output_ptr->points.emplace_back(point);
      }
    }
  }

  void twistCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    geometry_msgs::msg::Twist twist;

    if (!safety_check_) {
      if (0.0 < msg->linear.x)
        twist.linear.x = 0.0;
      else
        twist.linear.x = msg->linear.x;
      twist.angular.z = msg->angular.z;
    } else {
      twist.linear.x = msg->linear.x;
      twist.angular.z = msg->angular.z;
    }

    twist_publisher_->publish(twist);
  }

  void sensorCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr crop_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::fromROSMsg(*msg, *input_cloud);
    downsample(input_cloud, filtered_cloud, leaf_size_);
    crop(filtered_cloud, crop_cloud);

    if (crop_cloud->points.empty()) {
      safety_check_ = true;
      return;
    }

    crop_cloud->width = crop_cloud->points.size();
    crop_cloud->height = 1;
    // sensor_link to base_link
    transformPointCloud(
      crop_cloud, transformed_cloud, getTransform("base_link", msg->header.frame_id));

    bool safety_check = true;
    for (auto point : transformed_cloud->points) {
      point.z = 0;
      if (searchPolygon(point)) {
        safety_check = false;
        break;
      }
    }
    safety_check_ = safety_check;

    sensor_msgs::msg::PointCloud2 filtered_points_msg;
    pcl::toROSMsg(*transformed_cloud, filtered_points_msg);
    filtered_points_msg.header.frame_id = "base_link";
    filtered_points_msg.header.stamp = msg->header.stamp;
    filtered_points_publisher_->publish(filtered_points_msg);

    if (use_debug_) {
      geometry_msgs::msg::PolygonStamped polygon_stamped;
      polygon_stamped.header.frame_id = "base_link";
      polygon_stamped.header.stamp = msg->header.stamp;
      polygon_stamped.polygon = safety_polygon_;
      polygon_publisher_->publish(polygon_stamped);

      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = "base_link";
      marker.header.stamp = msg->header.stamp;
      marker.ns = "obstacle stop";
      marker.id = 0;

      marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      marker.action = visualization_msgs::msg::Marker::ADD;

      marker.scale.x = 0.5;
      marker.scale.y = 0.5;
      marker.scale.z = 0.5;
      marker.pose.position.x = -0.5;
      marker.pose.position.y = 0.0;
      marker.pose.position.z = 0.0;
      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0;
      marker.pose.orientation.w = 1.0;
      if (!safety_check_) {
        marker.text = "Obstacle Stop";
        marker.color.r = 1.0f;
        marker.color.g = 0.0f;
        marker.color.b = 0.0f;
      } else {
        marker.text = "No Obstacle";
        marker.color.r = 1.0f;
        marker.color.g = 1.0f;
        marker.color.b = 1.0f;
      }
      marker.color.a = 1.0f;
      marker_publisher_->publish(marker);
    }
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sensor_subscriber_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_subscriber_;
  rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr polygon_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_points_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher_;

  double leaf_size_;

  bool safety_check_{true};
  double use_debug_;

  double x_min_;
  double x_max_;
  double y_min_;
  double y_max_;
  double z_min_;
  double z_max_;

  geometry_msgs::msg::Polygon safety_polygon_;

  tf2_ros::Buffer tf2_buffer_{get_clock()};
  tf2_ros::TransformListener tf2_listener_{tf2_buffer_};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto obstacle_surround_stop_planner = std::make_shared<ObstacleSurroundStopPlanner>();
  rclcpp::spin(obstacle_surround_stop_planner);
  rclcpp::shutdown();
  return 0;
}
