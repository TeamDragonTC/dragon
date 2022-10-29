#ifndef _PARTICLE_FILTER_LOCALIZATION_HPP_
#define _PARTICLE_FILTER_LOCALIZATION_HPP_

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/transform_datatypes.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "particle_filter_localization/particle_filter.hpp"

class ParticleFilterLocalization : public rclcpp::Node
{
public:
  ParticleFilterLocalization();
  ~ParticleFilterLocalization() = default;

  void timerCallback();
  void callbackNDTPose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
  void callbackTwist(const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg);

private:
  void predict();
  void measure();
  void pubilshParticle(const rclcpp::Time stamp);
  geometry_msgs::msg::PoseStamped estimatedPose();
  void
  publishTF(const std::string frame_id, const std::string child_frame_id, const geometry_msgs::msg::PoseStamped pose);

private:
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr particle_pose_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr particle_array_publisher_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr ndt_pose_subscriber_;
  rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr twist_subscriber_;
  rclcpp::TimerBase::SharedPtr timer_;

  geometry_msgs::msg::PoseStamped current_pose_;
  geometry_msgs::msg::PoseWithCovarianceStamped current_pose_with_covariance_;

  geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr previous_twist_ptr_;

  std::shared_ptr<ParticleFilter> particle_filter_ptr_;

  tf2_ros::Buffer tf_buffer_{ get_clock() };
  tf2_ros::TransformListener tf_listener_{ tf_buffer_ };
  std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_;

  bool init_{ false };

  int particle_size_;
  int period_;
};

#endif
