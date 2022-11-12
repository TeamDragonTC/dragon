#include <pcl_conversions/pcl_conversions.h>
#include <geometry_msgs/msg/detail/vector3__struct.hpp>
#include <nav_msgs/msg/detail/occupancy_grid__struct.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

class ConvertPcdToGridMap : public rclcpp::Node
{
public:
  ConvertPcdToGridMap() : Node("convert_pcd_to_grid_map")
  {
    crop_max_z_ = this->declare_parameter<double>("crop_max_z");
    crop_min_z_ = this->declare_parameter<double>("crop_min_z");

    width_ = this->declare_parameter<double>("width");
    height_ = this->declare_parameter<double>("height");
    resolution_ = this->declare_parameter<double>("resolution");

    geometry_msgs::msg::Vector3 origin;
    origin.x = 0.0;
    origin.y = 0.0;
    origin.z = 0.0;
    initGridMap(origin);

    rclcpp::QoS qos{ 1 };
    qos.transient_local();
    qos.keep_last(1);

    map_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "points_map", rclcpp::QoS{ 1 }.transient_local(),
      std::bind(&ConvertPcdToGridMap::callbackMapPoints, this, std::placeholders::_1));
    grid_map_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("grid_map", qos);
    debug_points_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("debug_points", qos);
  }
  ~ConvertPcdToGridMap() = default;

  void initGridMap(const geometry_msgs::msg::Vector3 origin)
  {
    grid_map_.info.width = std::floor(width_ / resolution_);
    grid_map_.info.height = std::floor(height_ / resolution_);
    grid_map_.info.resolution = resolution_;
    grid_map_.data.resize(static_cast<int>(grid_map_.info.width * grid_map_.info.height));

    grid_map_.info.origin.position.x = origin.x - width_ / 2.0;
    grid_map_.info.origin.position.y = origin.y - height_ / 2.0;
  }

  void callbackMapPoints(const sensor_msgs::msg::PointCloud2& msg)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr map(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr ground_map(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr no_ground_map(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(msg, *map);

    for (int idx = 0; idx < grid_map_.data.size(); idx++)
      grid_map_.data[idx] = 50;
    grid_map_.header = msg.header;

    for (const auto& point : map->points) {
      // crop height
      // TODO: detect plane using other algorithm
      if (crop_min_z_ < point.z < crop_max_z_) {
        ground_map->points.emplace_back(point);
      } else {
        no_ground_map->points.emplace_back(point);
      }
    }
    for (const auto& point : ground_map->points) {
      const int ix = std::floor(point.x / resolution_) + grid_map_.info.width / 2;
      const int iy = std::floor(point.y / resolution_) + grid_map_.info.height / 2;

      if (grid_map_.info.width < ix or grid_map_.info.height < iy)
        continue;
      int grid_index = grid_map_.info.width * iy + ix;
      if (grid_map_.data.size() <= grid_index)
        continue;
      grid_map_.data[grid_index] = 0;
    }
    double max_height = 0.0;
    double min_height = 10000000.0;
    for (const auto& point : no_ground_map->points) {
      if (max_height < point.z)
        max_height = point.z;
      if (point.z < min_height)
        min_height = point.z;
    }

    for (const auto& point : no_ground_map->points) {
      const int ix = std::floor(point.x / resolution_) + grid_map_.info.width / 2;
      const int iy = std::floor(point.y / resolution_) + grid_map_.info.height / 2;

      if (grid_map_.info.width < ix or grid_map_.info.height < iy)
        continue;
      int grid_index = grid_map_.info.width * iy + ix;
      if (grid_map_.data.size() <= grid_index)
        continue;

      //double current_probability = static_cast<double>(grid_map_.data[grid_index] * 0.01);
      //double likelihood = (0.3 * current_probability);// + (0.4 * (1.0 - current_probability));
      double update_probability =
        ((point.z - min_height) / (5.0 - min_height));  //(0.3 * current_probability) / likelihood;
      if (update_probability < 0.0)
        update_probability = 0.0;
      if (1.0 < update_probability)
        update_probability = 1.0;
      grid_map_.data[grid_index] = 100;  //static_cast<int>(100.0 * update_probability);
    }

    sensor_msgs::msg::PointCloud2 debug_points_msg;
    pcl::toROSMsg(*no_ground_map, debug_points_msg);
    debug_points_msg.header = msg.header;
    debug_points_publisher_->publish(debug_points_msg);

    grid_map_publisher_->publish(grid_map_);
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr map_subscriber_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_map_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr debug_points_publisher_;

  nav_msgs::msg::OccupancyGrid grid_map_;

  double width_;
  double height_;
  double resolution_;

  double crop_min_z_;
  double crop_max_z_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<ConvertPcdToGridMap>());

  rclcpp::shutdown();

  return 0;
}
