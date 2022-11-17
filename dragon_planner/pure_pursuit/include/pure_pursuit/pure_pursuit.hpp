#ifndef _PURE_PURSUIT_H_
#define _PURE_PURSUIT_H_

#include <deque>
#include <fstream>
#include <string>
#include <sstream>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/bool.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <filesystem>
#include <rcpputils/filesystem_helper.hpp>

class PurePursuit : public rclcpp::Node
{
public:
  PurePursuit();
  ~PurePursuit() = default;

  void timerCallback();
  void callbackPose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
  void callbackCmd(const std_msgs::msg::Bool::SharedPtr msg);

private:
  bool loadWayPointFolder(const std::string waypoint_path);
  void loadWayPoint(const std::string waypoint_path);

  void publishWayPointMakrer(const std::deque<geometry_msgs::msg::Pose> waypoints);
  void publishTF(geometry_msgs::msg::Pose pose) const;
  void publishVelocity(double v, double w) const;
  double calcPurePursuit(geometry_msgs::msg::Pose pose, double& look_ahead);
  std::size_t planTargetPoint(geometry_msgs::msg::Pose pose, double& look_ahead);
  geometry_msgs::msg::Pose steeringControl(geometry_msgs::msg::Pose pose, double vel, double angle, double look_ahead);
  inline double pidVelocityControl(double target_vel, double current_vel)
  {
    return 1 * (target_vel - current_vel);
  }
  double quaternionToYaw(geometry_msgs::msg::Quaternion quat)
  {
    double roll, pitch, yaw;
    tf2::Quaternion tf_quat;
    tf_quat.setW(quat.w);
    tf_quat.setX(quat.x);
    tf_quat.setY(quat.y);
    tf_quat.setZ(quat.z);
    tf2::Matrix3x3(tf_quat).getRPY(roll, pitch, yaw);
    return yaw;
  }

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr cmd_subscriber_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_subscriber_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_subscriber_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;

  std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_;

  double target_w_{0.0};
  std::deque<std::string> waypoint_queue_;
  std::deque<geometry_msgs::msg::Pose> waypoint_list_;
  int target_waypoint_index_;
  geometry_msgs::msg::Pose closest_way_point_;
  geometry_msgs::msg::Pose target_way_point_;

  rclcpp::TimerBase::SharedPtr timer_;

  geometry_msgs::msg::Pose current_pose_;
  nav_msgs::msg::Path ref_path_;

  double goal_x_;
  double goal_y_;
  double goal_threshold_;
  bool flag_{ true };

  std::vector<double> ref_x_, ref_y_;
  double yaw_velocity_limit_;
  double current_vel_{ 0.0 };
  double yaw_vel_{ 0.0 };
  double target_vel_{ 0.0 };
  bool init_path_{ false };
  double period_;
};

#endif
