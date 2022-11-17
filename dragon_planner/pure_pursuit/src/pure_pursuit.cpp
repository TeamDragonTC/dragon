#include "pure_pursuit/pure_pursuit.hpp"
#include <filesystem>
#include <geometry_msgs/msg/detail/pose__struct.hpp>
#include <geometry_msgs/msg/detail/pose_with_covariance_stamped__struct.hpp>
#include <rclcpp/create_timer.hpp>
#include <string>
#include <visualization_msgs/msg/detail/marker__struct.hpp>

std::vector<std::string> split(std::string& input, char delimiter)
{
  std::istringstream stream(input);
  std::string field;
  std::vector<std::string> result;
  while (getline(stream, field, delimiter)) {
      result.push_back(field);
  }
  return result;
}

PurePursuit::PurePursuit() : Node("pure_pursuit")
{
  period_ = this->declare_parameter("period", 0.01);
  target_vel_ = this->declare_parameter("target_vel", 3.0);
  goal_threshold_ = this->declare_parameter("goal_threshold", 0.2);
  yaw_velocity_limit_ = this->declare_parameter<double>("yaw_velocity_limit");

  target_waypoint_index_ = this->declare_parameter<int>("target_way_point");
  std::string waypoint_path = this->declare_parameter<std::string>("waypoint_path");
 
  broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

  velocity_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

  rclcpp::QoS qos{1};
  qos.transient_local();
  qos.keep_last(1);
  marker_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("marker", qos);

  cmd_subscriber_ = this->create_subscription<std_msgs::msg::Bool>(
    "cmd", 1, std::bind(&PurePursuit::callbackCmd, this, std::placeholders::_1));
  pose_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "pose_with_covariance", 5, std::bind(&PurePursuit::callbackPose, this, std::placeholders::_1));

  loadWayPointFolder(waypoint_path);
  // load first waypoints
  loadWayPoint(waypoint_queue_.front());
  waypoint_queue_.pop_front();

  timer_ = rclcpp::create_timer(
    this, get_clock(), rclcpp::Duration::from_seconds(period_), std::bind(&PurePursuit::timerCallback, this));
}

bool PurePursuit::loadWayPointFolder(const std::string waypoint_path)
{
  std::deque<std::string> waypoint_queue;
  rcpputils::fs::path dir(waypoint_path);
  if(!dir.exists()) {
    RCLCPP_ERROR(get_logger(), "can not open directory: %s", waypoint_path.c_str());
    return false;
  }

  for(const auto & file : std::filesystem::directory_iterator(waypoint_path)) {
    const std::string filename = file.path().c_str();
    const std::string extension = file.path().extension().string();
    if(extension != ".csv") {
      RCLCPP_WARN(get_logger(), "ignore files: %s", extension.c_str());
      continue;
    }
    waypoint_queue.emplace_back(filename);
  }

  if(waypoint_queue.empty()) {
    RCLCPP_ERROR(get_logger(), "waypoint queue is Zero.");
    return false;
  }

  // sort file
  for(std::size_t idx=0;idx<waypoint_queue.size(); idx++) {
    for(auto file : waypoint_queue) {
      if(file.find(std::to_string(idx)) != std::string::npos) {
        waypoint_queue_.emplace_back(file);
        break;
      }
    }
  }

  return true;
}

void PurePursuit::loadWayPoint(const std::string waypoint_path)
{
  std::ifstream ifs(waypoint_path);

  waypoint_list_.clear();

  std::string line;
  int count = 0;
  while(std::getline(ifs, line))
  {
    std::vector<std::string> vec = split(line, ',');

    count++;

    if(count == 1) continue;
    geometry_msgs::msg::Pose pose;
    pose.position.x = std::stod(vec[1]);
    pose.position.y = std::stod(vec[2]);
    pose.position.z = std::stod(vec[3]);
    pose.orientation.x = std::stod(vec[4]);
    pose.orientation.y = std::stod(vec[5]);
    pose.orientation.z = std::stod(vec[6]);
    pose.orientation.w = std::stod(vec[7]);
    waypoint_list_.emplace_back(pose);
  }

  if(!waypoint_list_.empty()) {
    init_path_ = true;

    goal_x_ = waypoint_list_.back().position.x;
    goal_y_ = waypoint_list_.back().position.y;

    publishWayPointMakrer(waypoint_list_);
  }
}

void PurePursuit::publishWayPointMakrer(const std::deque<geometry_msgs::msg::Pose> waypoints)
{
  visualization_msgs::msg::MarkerArray marker_array;
  for(std::size_t idx=0;idx<waypoints.size(); idx++) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = now();
    marker.ns = "waypoint";
    marker.id = idx++;
    marker.pose = waypoints[idx];
    marker.text = std::to_string(idx);
    marker.type = visualization_msgs::msg::Marker::CYLINDER;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.scale.z = 0.1;
    marker.color.a = 0.7;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker_array.markers.emplace_back(marker);
  }
  marker_publisher_->publish(marker_array);
}

void PurePursuit::callbackCmd(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (init_path_) {
    flag_ = msg->data;
  } else {
    RCLCPP_ERROR(get_logger(), "Path is Not Set.");
  }
}

void PurePursuit::callbackPose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  current_pose_ = msg->pose.pose;

  // search closest way point
  std::size_t closest_way_point_index = -1;
  double min_distance = 100000000.0;
  for(std::size_t idx = 0; idx < waypoint_list_.size(); idx++) {
    double dx = current_pose_.position.x - waypoint_list_[idx].position.x;
    double dy = current_pose_.position.y - waypoint_list_[idx].position.y;
    double distance = std::sqrt(dx * dx + dy * dy);
    if(distance < min_distance) {
      min_distance = distance;
      closest_way_point_index = idx;
    }
  }
  std::size_t base_waypoint_index = closest_way_point_index + target_waypoint_index_;
  if(waypoint_list_.size() - 1 < base_waypoint_index)
    base_waypoint_index = waypoint_list_.size() - 1;
  target_way_point_ = waypoint_list_[base_waypoint_index];
}

void PurePursuit::timerCallback()
{
  if (flag_ or !init_path_)
    return;

  static double prev_steering_angle = 0.0;
  static auto prev_stamp = now();
  auto current_stamp = now();
  const double dt = (rclcpp::Time(current_stamp) - rclcpp::Time(prev_stamp)).seconds();

  double look_ahead;
  const double ak = pidVelocityControl(target_vel_, current_vel_);
  const double steering_angle = calcPurePursuit(current_pose_, look_ahead);
  current_pose_ = steeringControl(current_pose_, ak, steering_angle, look_ahead);
  yaw_vel_ = (steering_angle - prev_steering_angle) / dt;
  publishVelocity(current_vel_, target_w_);

  prev_stamp = current_stamp;
  prev_steering_angle = steering_angle;

  const double dx = goal_x_ - current_pose_.position.x;
  const double dy = goal_y_ - current_pose_.position.y;
  const double dist = std::sqrt(dx * dx + dy * dy);

  if (dist < goal_threshold_) {
    RCLCPP_INFO(get_logger(), "goal reached.");
    init_path_ = false;
    flag_ = true;
    // load next waypoint list
    if(!waypoint_queue_.empty()) {
      RCLCPP_INFO(get_logger(), "Set Next Way Point: %s", waypoint_queue_.front().c_str());
      loadWayPoint(waypoint_queue_.front());
      waypoint_queue_.pop_front();
    }
    publishVelocity(0.0, 0.0);
  }
}

void PurePursuit::publishVelocity(double v, double w) const
{
  geometry_msgs::msg::Twist vel;
  vel.linear.x = v;
  vel.angular.z = w;
  velocity_publisher_->publish(vel);
}

// tfを出力
void PurePursuit::publishTF(geometry_msgs::msg::Pose pose) const
{
  geometry_msgs::msg::TransformStamped transformStamped;

  transformStamped.header.stamp = rclcpp::Clock().now();
  transformStamped.header.frame_id = "map";
  transformStamped.child_frame_id = "pure_base_link";
  transformStamped.transform.translation.x = pose.position.x;
  transformStamped.transform.translation.y = pose.position.y;
  transformStamped.transform.translation.z = pose.position.z;
  transformStamped.transform.rotation.x = pose.orientation.x;
  transformStamped.transform.rotation.y = pose.orientation.y;
  transformStamped.transform.rotation.z = pose.orientation.z;
  transformStamped.transform.rotation.w = pose.orientation.w;

  broadcaster_->sendTransform(transformStamped);
}

// pure pursuitによるステアリング角の決定
double PurePursuit::calcPurePursuit(geometry_msgs::msg::Pose pose, double& look_ahead)
{
  const double yaw = quaternionToYaw(pose.orientation);
  double dx = target_way_point_.position.x - pose.position.x;
  double dy = target_way_point_.position.y - pose.position.y;
  const double alpha = std::atan2(dy, dx) - yaw;
  //const double alpha = std::atan2(ref_y_[index] - pose.position.y, ref_x_[index] - pose.position.x) - yaw;
  look_ahead = std::sqrt(dx * dx + dy * dy);
  const double steering_angle = std::atan2(2.0 * look_ahead * std::sin(alpha) / (0.3 + 0.1 * current_vel_), 1.0);
  target_w_ = 2 * current_vel_ * std::sin(alpha) / look_ahead;
  if(yaw_velocity_limit_ < target_w_)
    target_w_ = yaw_velocity_limit_;
  if(target_w_ < -1 * yaw_velocity_limit_)
    target_w_ = -1 * yaw_velocity_limit_;
  return steering_angle;
}

// look ahead distanceの更新とターゲット位置を取得
std::size_t PurePursuit::planTargetPoint(geometry_msgs::msg::Pose pose, double& look_ahead)
{
  std::vector<double> dx, dy;

  for (std::size_t index = 0; index < ref_x_.size(); index++) {
    dx.push_back(pose.position.x - ref_x_[index]);
    dy.push_back(pose.position.y - ref_y_[index]);
  }
  std::vector<double> distance;
  for (std::size_t index = 0; index < dx.size(); index++) {
    const double dist = std::abs(std::sqrt(dx[index] * dx[index] + dy[index] * dy[index]));
    distance.push_back(dist);
  }
  std::vector<double>::iterator iter = std::min_element(distance.begin(), distance.end());
  std::size_t index = std::distance(distance.begin(), iter);
  const double look_ahead_filter = 0.1 * current_vel_ + 0.3;
  look_ahead = 0.0;
  // look ahead distanceの更新
  while (look_ahead_filter > look_ahead && (index + 1) < ref_x_.size()) {
    const double d_x = ref_x_[index + 1] - ref_x_[index];
    const double d_y = ref_y_[index + 1] - ref_y_[index];
    look_ahead += std::sqrt(d_x * d_x + d_y * d_y);
    index += 1;
  }
  return index;
}

// 現在位置と速度、ステアンリング各からt+1後のロボットの位置を計算する
geometry_msgs::msg::Pose
PurePursuit::steeringControl(geometry_msgs::msg::Pose pose, double vel, double angle, double look_ahead)
{
  geometry_msgs::msg::Pose update_pose = pose;
  // 現在姿勢からyaw軸の姿勢を取得
  double yaw = quaternionToYaw(pose.orientation);
  update_pose.position.x = pose.position.x + current_vel_ * std::cos(yaw) * 0.01;
  update_pose.position.y = pose.position.y + current_vel_ * std::sin(yaw) * 0.01;
  yaw += (current_vel_ / look_ahead) * std::tan(angle) * (0.01);
  current_vel_ += vel * 0.01;
  tf2::Quaternion quat;
  quat.setRPY(0.0, 0.0, yaw);
  update_pose.orientation.w = quat.w();
  update_pose.orientation.x = quat.x();
  update_pose.orientation.y = quat.y();
  update_pose.orientation.z = quat.z();
  return update_pose;
}
