#include <fast_gicp/gicp/fast_gicp.hpp>
#include <icp_localization/icp_localization.hpp>

ICPLocalization::ICPLocalization() : Node("icp_localization")
{
  min_range_ = this->declare_parameter<double>("min_range");
  max_range_ = this->declare_parameter<double>("max_range");
  downsample_leaf_size_ = this->declare_parameter<double>("downsample_leaf_size");
  transformation_epsilon_ = this->declare_parameter<double>("transformation_epsilon");
  max_correspondence_distance_ = this->declare_parameter<double>("max_correspondence_distance");
  euclidean_fitness_epsilon_ = this->declare_parameter<double>("euclidean_fitness_epsilon");
  ransac_outlier_rejection_threshold_ = this->declare_parameter<double>("ransac_outlier_rejection_threshold");
  max_iteration_ = this->declare_parameter<int>("max_iteration");
  map_frame_id_ = this->declare_parameter<std::string>("map_frame_id");
  base_frame_id_ = this->declare_parameter<std::string>("base_frame_id");

  broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

  const std::string registration_type = this->declare_parameter<std::string>("registration_type");
  if (registration_type == "FAST_GICP") {
    std::shared_ptr<fast_gicp::FastGICP<PointType, PointType>> fast_gicp(new fast_gicp::FastGICP<PointType, PointType>);
    const int num_thread = this->declare_parameter<int>("gicp_num_thread");
    if (0 < num_thread)
      fast_gicp->setNumThreads(num_thread);
    registration_ = fast_gicp;
  } else {
    std::shared_ptr<pcl::GeneralizedIterativeClosestPoint<PointType, PointType>> gicp(
      new pcl::GeneralizedIterativeClosestPoint<PointType, PointType>);
    registration_ = gicp;
  }
  registration_->setMaximumIterations(max_iteration_);
  registration_->setTransformationEpsilon(transformation_epsilon_);
  registration_->setMaxCorrespondenceDistance(max_correspondence_distance_);
  registration_->setEuclideanFitnessEpsilon(euclidean_fitness_epsilon_);
  registration_->setRANSACOutlierRejectionThreshold(ransac_outlier_rejection_threshold_);

  map_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "points_map", rclcpp::QoS{ 1 }.transient_local(),
    std::bind(&ICPLocalization::mapCallback, this, std::placeholders::_1));
  points_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "points_raw", rclcpp::SensorDataQoS().keep_last(5),
    std::bind(&ICPLocalization::pointsCallback, this, std::placeholders::_1));
  initialpose_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "initialpose", 1, std::bind(&ICPLocalization::initialPoseCallback, this, std::placeholders::_1));

  icp_align_cloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("aligned_cloud", 1);
  icp_pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("icp_pose", 5);
  icp_pose_with_covariance_publisher_ =
    this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("icp_pose_with_covariance", 5);
}

void ICPLocalization::downsample(
  const pcl::PointCloud<PointType>::Ptr& input_cloud_ptr, pcl::PointCloud<PointType>::Ptr& output_cloud_ptr)
{
  pcl::VoxelGrid<PointType> voxel_grid;
  voxel_grid.setLeafSize(downsample_leaf_size_, downsample_leaf_size_, downsample_leaf_size_);
  voxel_grid.setInputCloud(input_cloud_ptr);
  voxel_grid.filter(*output_cloud_ptr);
}

void ICPLocalization::crop(
  const pcl::PointCloud<PointType>::Ptr& input_cloud_ptr, pcl::PointCloud<PointType>::Ptr output_cloud_ptr,
  const double min_range, const double max_range)
{
  for (const auto& p : input_cloud_ptr->points) {
    const double dist = std::sqrt(p.x * p.x + p.y * p.y);
    if (min_range < dist && dist < max_range) {
      output_cloud_ptr->points.emplace_back(p);
    }
  }
}

void ICPLocalization::mapCallback(const sensor_msgs::msg::PointCloud2& map)
{
  RCLCPP_INFO(get_logger(), "map callback");

  pcl::PointCloud<PointType>::Ptr map_cloud(new pcl::PointCloud<PointType>);
  pcl::fromROSMsg(map, *map_cloud);

  registration_->setInputTarget(map_cloud);
}

void ICPLocalization::pointsCallback(const sensor_msgs::msg::PointCloud2& points)
{
  if (registration_->getInputTarget() == nullptr) {
    RCLCPP_ERROR(get_logger(), "map not received!");
    return;
  }

  if (!localization_ready_) {
    RCLCPP_ERROR(get_logger(), "initial pose not received!");
    return;
  }

  const rclcpp::Time current_scan_time = points.header.stamp;

  pcl::PointCloud<PointType>::Ptr input_cloud_ptr(new pcl::PointCloud<PointType>);
  pcl::fromROSMsg(points, *input_cloud_ptr);

  // downsampling input point cloud
  pcl::PointCloud<PointType>::Ptr filtered_cloud(new pcl::PointCloud<PointType>);
  downsample(input_cloud_ptr, filtered_cloud);

  // crop point cloud
  pcl::PointCloud<PointType>::Ptr crop_cloud(new pcl::PointCloud<PointType>);
  crop(filtered_cloud, crop_cloud, min_range_, max_range_);

  crop_cloud->width = crop_cloud->points.size();
  crop_cloud->height = 1;

  // transform base_link to sensor_link
  pcl::PointCloud<PointType>::Ptr transform_cloud_ptr(new pcl::PointCloud<PointType>);
  const std::string sensor_frame_id = points.header.frame_id;
  geometry_msgs::msg::TransformStamped sensor_frame_transform;
  try {
    sensor_frame_transform =
      tf_buffer_.lookupTransform(base_frame_id_, sensor_frame_id, current_scan_time, tf2::durationFromSec(0.5));
  } catch (tf2::TransformException& ex) {
    RCLCPP_ERROR(get_logger(), "%s", ex.what());
    sensor_frame_transform.header.stamp = current_scan_time;
    sensor_frame_transform.header.frame_id = base_frame_id_;
    sensor_frame_transform.child_frame_id = sensor_frame_id;
    sensor_frame_transform.transform.translation.x = 0.0;
    sensor_frame_transform.transform.translation.y = 0.0;
    sensor_frame_transform.transform.translation.z = 0.0;
    sensor_frame_transform.transform.rotation.w = 1.0;
    sensor_frame_transform.transform.rotation.x = 0.0;
    sensor_frame_transform.transform.rotation.y = 0.0;
    sensor_frame_transform.transform.rotation.z = 0.0;
  }
  const Eigen::Affine3d base_to_sensor_frame_affine = tf2::transformToEigen(sensor_frame_transform);
  const Eigen::Matrix4f base_to_sensor_frame_matrix = base_to_sensor_frame_affine.matrix().cast<float>();
  pcl::transformPointCloud(*crop_cloud, *transform_cloud_ptr, base_to_sensor_frame_matrix);
  registration_->setInputSource(transform_cloud_ptr);

  // calculation initial pose for icp
  Eigen::Matrix4f init_guess = Eigen::Matrix4f::Identity();
  Eigen::Affine3d initial_pose_affine;
  tf2::fromMsg(initial_pose_, initial_pose_affine);
  init_guess = initial_pose_affine.matrix().cast<float>();

  pcl::PointCloud<PointType>::Ptr output_cloud(new pcl::PointCloud<PointType>);
  registration_->align(*output_cloud, init_guess);

  const bool convergenced = registration_->hasConverged();

  const Eigen::Matrix4f result_icp_pose = registration_->getFinalTransformation();

  Eigen::Affine3d result_icp_pose_affine;
  result_icp_pose_affine.matrix() = result_icp_pose.cast<double>();
  const geometry_msgs::msg::Pose icp_pose = tf2::toMsg(result_icp_pose_affine);
  initial_pose_ = icp_pose;

  geometry_msgs::msg::PoseStamped icp_pose_msg;
  icp_pose_msg.header.frame_id = map_frame_id_;
  icp_pose_msg.header.stamp = current_scan_time;
  icp_pose_msg.pose = icp_pose;

  if (convergenced) {
    icp_pose_publisher_->publish(icp_pose_msg);
    publishTF(map_frame_id_, base_frame_id_, icp_pose_msg);
  }

  sensor_msgs::msg::PointCloud2 aligned_cloud_msg;
  pcl::toROSMsg(*output_cloud, aligned_cloud_msg);
  aligned_cloud_msg.header = points.header;
  icp_align_cloud_publisher_->publish(aligned_cloud_msg);
}

void ICPLocalization::initialPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped& initialpose)
{
  RCLCPP_INFO(get_logger(), "initial pose callback.");
  if (initialpose.header.frame_id == map_frame_id_) {
    initial_pose_ = initialpose.pose.pose;
    if (!localization_ready_)
      localization_ready_ = true;
  } else {
    // TODO transform
    RCLCPP_ERROR(
      get_logger(), "frame_id is not same. initialpose.header.frame_id is %s", initialpose.header.frame_id.c_str());
  }
}

void ICPLocalization::publishTF(
  const std::string frame_id, const std::string child_frame_id, const geometry_msgs::msg::PoseStamped pose)
{
  geometry_msgs::msg::TransformStamped transform_stamped;

  transform_stamped.header.frame_id = frame_id;
  transform_stamped.header.stamp = pose.header.stamp;
  transform_stamped.child_frame_id = child_frame_id;
  transform_stamped.transform.translation.x = pose.pose.position.x;
  transform_stamped.transform.translation.y = pose.pose.position.y;
  transform_stamped.transform.translation.z = pose.pose.position.z;
  transform_stamped.transform.rotation.w = pose.pose.orientation.w;
  transform_stamped.transform.rotation.x = pose.pose.orientation.x;
  transform_stamped.transform.rotation.y = pose.pose.orientation.y;
  transform_stamped.transform.rotation.z = pose.pose.orientation.z;

  broadcaster_->sendTransform(transform_stamped);
}
