#include <fstream>
#include <std_msgs/msg/detail/color_rgba__struct.hpp>
#include <waypoint_generator/waypoint_generator.hpp>

WayPointGenerator::WayPointGenerator() : Node("waypoint_generator")
{
  way_point_csv_path_ = this->declare_parameter("way_point_csv_path", "");
  threshold_ = this->declare_parameter("threshold", 0.0);

  way_point_csv_file_name_ = "way_point_" + std::to_string(way_point_csv_index_) + ".csv";

  joy_subscriber_ = this->create_subscription<sensor_msgs::msg::Joy>("joy", 5, std::bind(&WayPointGenerator::joyCallback, this, std::placeholders::_1));
  pose_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "pose_with_covariance", 5, std::bind(&WayPointGenerator::poseCallback, this, std::placeholders::_1));

  waypoint_marker_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("waypoint_marker", 5);

  std::ofstream ofs(way_point_csv_path_ + "/" + way_point_csv_file_name_);
  // clang-format off
  ofs << "way point ID" << "," 
      << "pose.position.x" << ","
      << "pose.position.y" << ","
      << "pose.position.z" << ","
      << "pose.orientation.x" << ","
      << "pose.orientation.y" << ","
      << "pose.orientation.z" << ","
      << "pose.orientation.w" << ","
      << "target velocity" << std::endl;
  // clang-format on
}

void WayPointGenerator::joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
  static auto prev_joy_msg = msg;
  if(prev_joy_msg->buttons[1] == 0 and msg->buttons[1] == 1) {
    way_point_csv_file_name_ = "way_point_" + std::to_string(++way_point_csv_index_) + ".csv";
    std::ofstream ofs(way_point_csv_path_ + "/" + way_point_csv_file_name_);
    // clang-format off
    ofs << "way point ID" << "," 
        << "pose.position.x" << ","
        << "pose.position.y" << ","
        << "pose.position.z" << ","
        << "pose.orientation.x" << ","
        << "pose.orientation.y" << ","
        << "pose.orientation.z" << ","
        << "pose.orientation.w" << ","
        << "target velocity" << std::endl;
    // clang-format on
  }
  prev_joy_msg = msg;
}

double WayPointGenerator::getDist(const geometry_msgs::msg::Pose pose_1, const geometry_msgs::msg::Pose pose_2)
{
  const double dx = pose_1.position.x - pose_2.position.x;
  const double dy = pose_1.position.y - pose_2.position.y;
  const double distance = std::sqrt(dx * dx + dy * dy);
  return distance;
}

double WayPointGenerator::getVelocity(
  const geometry_msgs::msg::Pose pose_1, const geometry_msgs::msg::Pose pose_2, const double dt)
{
  const double dx = pose_1.position.x - pose_2.position.x;
  const double dy = pose_1.position.y - pose_2.position.y;
  const double distance = std::sqrt(dx * dx + dy * dy);
  return distance / dt;
}

geometry_msgs::msg::Vector3 WayPointGenerator::createScale(const double x, const double y, const double z)
{
  geometry_msgs::msg::Vector3 vector3;
  vector3.x = x;
  vector3.y = y;
  vector3.z = z;
  return vector3;
}

std_msgs::msg::ColorRGBA WayPointGenerator::createColor(const double a, const double r, const double g, const double b)
{
  std_msgs::msg::ColorRGBA color;
  color.a = a;
  color.r = r;
  color.g = g;
  color.b = b;
  return color;
}

visualization_msgs::msg::Marker WayPointGenerator::createMarker(
  const geometry_msgs::msg::Pose pose, const int32_t type, const int id, const std::string text,
  geometry_msgs::msg::Vector3 scale, std_msgs::msg::ColorRGBA color)
{
  visualization_msgs::msg::Marker marker;

  marker.header.frame_id = "map";
  marker.header.stamp = now();
  marker.ns = "way_point";
  marker.id = id;
  marker.type = type;
  marker.text = text;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose = pose;
  marker.scale = scale;
  marker.color = color;

  return marker;
}

void WayPointGenerator::poseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  static geometry_msgs::msg::PoseWithCovarianceStamped prev_pose = *msg;
  static geometry_msgs::msg::PoseWithCovarianceStamped prev_way_point = *msg;

  const double dt = (rclcpp::Time(msg->header.stamp) - rclcpp::Time(prev_pose.header.stamp)).seconds();

  double distance = getDist(msg->pose.pose, prev_way_point.pose.pose);
  double linear_velocity = getVelocity(msg->pose.pose, prev_pose.pose.pose, dt);
  prev_pose = *msg;

  if (distance < threshold_ and id_ != 0)
    return;
  prev_way_point = *msg;

  if (id_ == 0)
    linear_velocity = 0.0;

  std::ofstream ofs(way_point_csv_path_ + "/" + way_point_csv_file_name_, std::ios::app);
  // clang-format off
  ofs << way_point_id_++ << ","
      << msg->pose.pose.position.x << ","
      << msg->pose.pose.position.y << ","
      << msg->pose.pose.position.z << ","
      << msg->pose.pose.orientation.x << ","
      << msg->pose.pose.orientation.y << ","
      << msg->pose.pose.orientation.z << ","
      << msg->pose.pose.orientation.w << ","
      << linear_velocity << std::endl;
  // clang-format on

  waypoint_marker_publisher_->publish(createMarker(
    msg->pose.pose, visualization_msgs::msg::Marker::CYLINDER, id_++, std::to_string(way_point_id_),
    createScale(0.2, 0.2, 0.1), createColor(0.7, 0.0, 1.0, 0.0)));
  waypoint_marker_publisher_->publish(createMarker(
    msg->pose.pose, visualization_msgs::msg::Marker::TEXT_VIEW_FACING, id_++, "ID:" + std::to_string(way_point_id_),
    createScale(0.3, 0.3, 0.3), createColor(1.0, 1.0, 1.0, 1.0)));
}
