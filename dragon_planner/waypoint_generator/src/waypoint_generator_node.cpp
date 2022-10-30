#include <waypoint_generator/waypoint_generator.hpp>

int main(int argc, char**argv)
{
  rclcpp::init(argc, argv);
  auto waypoint_generator = std::make_shared<WayPointGenerator>();
  rclcpp::spin(waypoint_generator);
  rclcpp::shutdown();
  return 0;
}
