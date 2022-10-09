#include <dragon_control/velocity_smoother.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VelocitySmoother>());
  rclcpp::shutdown();
  return 0;
}
