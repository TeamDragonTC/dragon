#include <pcl/impl/point_types.hpp>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/detail/occupancy_grid__struct.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/detail/point_cloud2__struct.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#ifdef USE_TF2_GEOMETRY_MSGS_DEPRECATED_HEADER
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_eigen/tf2_eigen.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif

#include <pcl_ros/transforms.hpp>

#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/buffer.h>

class OccupancyGridMap : public rclcpp::Node
{
public:
  OccupancyGridMap() : Node("occupancy_grid_map")
  {
    width_ = this->declare_parameter("width", 30.0);
    height_ = this->declare_parameter("height", 30.0);
    resolution_ = this->declare_parameter("resolution", 0.1);

    center_x_ = center_y_ = 0.0;

    sensor_points_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "velodyne_points", rclcpp::SensorDataQoS().keep_last(1),
      std::bind(&OccupancyGridMap::sensorCallback, this, std::placeholders::_1));
    occupancy_grid_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("grid_map", 5);
  }
  ~OccupancyGridMap() = default;

  geometry_msgs::msg::TransformStamped getTransform(
    const std::string target_frame, const std::string source_frame, rclcpp::Time stamp)
  {
    geometry_msgs::msg::TransformStamped frame_transform;
    try {
      frame_transform =
        tf2_buffer_.lookupTransform(target_frame, source_frame, stamp, tf2::durationFromSec(0.5));
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

  void sensorCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::fromROSMsg(*msg, *input_cloud);

    transformPointCloud(
      input_cloud, transformed_cloud,
      getTransform("base_link", msg->header.frame_id, msg->header.stamp));

    nav_msgs::msg::OccupancyGrid grid_map;
    grid_map.info.width = width_ / resolution_;
    grid_map.info.height = height_ / resolution_;
    grid_map.info.resolution = resolution_;

    int cell_size = static_cast<int>((width_ / resolution_) * (height_ / resolution_));
    for (int idx = 0; idx < cell_size; idx++) {
      grid_map.data.emplace_back(50);
    }

    grid_map.info.origin.position.x = center_x_ - width_ / 2.0;
    grid_map.info.origin.position.y = center_y_ - height_ / 2.0;

    grid_map.header.stamp = msg->header.stamp;
    grid_map.header.frame_id = "base_link";

    for (auto & point : transformed_cloud->points) {
      if (point.z < 0.5) continue;

      const auto x = point.x - grid_map.info.origin.position.x;
      const auto y = point.y - grid_map.info.origin.position.y;
      const int ix = static_cast<int>(std::floor(x / resolution_));
      const int iy = static_cast<int>(std::floor(y / resolution_));

      int grid_index = grid_map.info.width * iy + ix;
      if (cell_size <= grid_index) continue;
      grid_map.data[grid_index] = 100;
    }

    occupancy_grid_publisher_->publish(grid_map);
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sensor_points_subscriber_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_grid_publisher_;

  double width_;
  double height_;
  double resolution_;
  double center_x_;
  double center_y_;
  std::vector<double> data_;

  tf2_ros::Buffer tf2_buffer_{get_clock()};
  tf2_ros::TransformListener tf2_listener_{tf2_buffer_};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto occupancy_grid_map = std::make_shared<OccupancyGridMap>();
  rclcpp::spin(occupancy_grid_map);
  rclcpp::shutdown();
  return 0;
}
