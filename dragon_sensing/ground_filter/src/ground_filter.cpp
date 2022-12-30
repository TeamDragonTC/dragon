#include "ground_filter/ground_filter.hpp"

GroundFilter::GroundFilter() : Node("ground_filter")
{
  bottom_ring_ = this->declare_parameter<int>("bottom_ring");
  angle_threshold_ = this->declare_parameter<double>("angle_threshold");
  height_threshold_ = this->declare_parameter<double>("height_threshold");
  lidar_model_.resolution = this->declare_parameter<double>("resolution");
  lidar_model_.line = this->declare_parameter<int>("line");
  lidar_model_.fov_up = radian(this->declare_parameter<double>("fov_up"));
  lidar_model_.fov_down = radian(this->declare_parameter<double>("fov_down"));
  lidar_model_.fov = std::fabs(lidar_model_.fov_up) + std::fabs(lidar_model_.fov_down);

  sensor_points_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "points_raw", rclcpp::SensorDataQoS().keep_last(1),
    std::bind(&GroundFilter::callbackSensorPoints, this, std::placeholders::_1));

  no_ground_publisher_ =
    this->create_publisher<sensor_msgs::msg::PointCloud2>("no_ground_points", 5);
  ground_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("ground_points", 5);

  sensor_image_publisher_ = image_transport::create_publisher(this, "sensor_image");
}

geometry_msgs::msg::TransformStamped GroundFilter::getTransform(
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

void GroundFilter::transformPointCloud(
  pcl::PointCloud<PointType>::Ptr input_ptr, pcl::PointCloud<PointType>::Ptr & output_ptr,
  const geometry_msgs::msg::TransformStamped frame_transform)
{
  const Eigen::Affine3d frame_affine = tf2::transformToEigen(frame_transform);
  const Eigen::Matrix4f frame_matrix = frame_affine.matrix().cast<float>();
  pcl::transformPointCloud(*input_ptr, *output_ptr, frame_matrix);
}

void GroundFilter::callbackSensorPoints(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  pcl::PointCloud<PointType>::Ptr input_points(new pcl::PointCloud<PointType>);
  pcl::PointCloud<PointType>::Ptr projection_points(new pcl::PointCloud<PointType>);
  pcl::PointCloud<PointType>::Ptr ground_points(new pcl::PointCloud<PointType>);
  pcl::PointCloud<PointType>::Ptr no_ground_points(new pcl::PointCloud<PointType>);

  pcl::fromROSMsg(*msg, *input_points);

  int width = static_cast<int>(360.0 / lidar_model_.resolution);
  int height = lidar_model_.line;

  projection_points->points.resize(width * height);

  cv::Mat sensor_image(height, width, CV_32FC1, cv::Scalar(-1));

  // convert range image
  for (const auto & p : input_points->points) {
    double range = std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z);

    int pixel_x = -1;
    int pixel_y = -1;

    const double yaw = std::atan2(p.y, p.x);
    const double pitch = std::asin(p.z / range);

    pixel_x = static_cast<int>(std::floor(width * (0.5 * (1.0 + yaw / M_PI))));
    pixel_x = std::min(std::max(pixel_x, 0), width - 1);

    pixel_y = static_cast<int>(
      std::floor(height * (1.0 - (pitch + lidar_model_.fov_up) / lidar_model_.fov)));

    pixel_y = std::min(std::max(pixel_y, 0), height - 1);

    sensor_image.at<float>(pixel_y, pixel_x) = range;

    std::size_t idx = pixel_x + pixel_y * width;
    projection_points->points[idx] = p;
  }

  #if 1
  // ground remove
  for (std::size_t x = 0; x < width; ++x) {
    for (int y = (lidar_model_.line - 1); bottom_ring_ < y; --y) {
      std::size_t curr_idx = x + y * width;
      std::size_t next_idx = x + (y - 1) * width;

      const double dx =
        projection_points->points[next_idx].x - projection_points->points[curr_idx].x;
      const double dy =
        projection_points->points[next_idx].y - projection_points->points[curr_idx].y;
      const double dz =
        projection_points->points[next_idx].z - projection_points->points[curr_idx].z;

      // angle check
      const double angle = degree(std::atan2(dz, std::sqrt(dx * dx + dy * dy)));
      const double height = std::sqrt(dx * dx + dy * dy + dz * dz);
      if (std::fabs(angle) <= angle_threshold_) {
        ground_points->points.emplace_back(projection_points->points[curr_idx]);
        ground_points->points.emplace_back(projection_points->points[next_idx]);
      } else {
        no_ground_points->points.emplace_back(projection_points->points[curr_idx]);
        ground_points->points.emplace_back(projection_points->points[next_idx]);
      }
    }
  }
  #endif
  bool start = true;
  std::size_t k = 0;
#if 0
  for(std::size_t x = 0; x < width; x++) {
    for(std::size_t y = 0; y < lidar_model_.line; y++) {
      std::size_t curr_idx = x + y * width;
      if(start) {
        ground_points->points.emplace_back(projection_points->points[curr_idx]);
        k = y;
        start = false;
      } else {
        std::size_t next_idx = x + k * width;
        const double dx = projection_points->points[curr_idx].x - projection_points->points[next_idx].x;
        const double dy = projection_points->points[curr_idx].y - projection_points->points[next_idx].y;
        const double dz = projection_points->points[curr_idx].z - projection_points->points[next_idx].z;

        const double angle = degree(std::atan2(dz, std::sqrt(dx * dx + dy * dy)));
        const double height = std::sqrt(dx * dx + dy * dy + dz * dz);
        if (std::fabs(angle) <= angle_threshold_ and std::fabs(height) <= height_threshold_) {
          ground_points->points.emplace_back(projection_points->points[curr_idx]);
          k = y;
        }
      }
    }
  }
#endif
  sensor_msgs::msg::PointCloud2 ground_points_msg;
  pcl::toROSMsg(*ground_points, ground_points_msg);
  ground_points_msg.header = msg->header;
  ground_publisher_->publish(ground_points_msg);

  sensor_msgs::msg::PointCloud2 no_ground_points_msg;
  pcl::toROSMsg(*no_ground_points, no_ground_points_msg);
  no_ground_points_msg.header = msg->header;
  no_ground_publisher_->publish(no_ground_points_msg);

  sensor_msgs::msg::Image::SharedPtr image_msgs =
    cv_bridge::CvImage(msg->header, sensor_msgs::image_encodings::TYPE_32FC1, sensor_image)
      .toImageMsg();
  image_msgs->header = msg->header;
  sensor_image_publisher_.publish(image_msgs);
}