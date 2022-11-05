#include "pure_pursuit/pure_pursuit.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto pure_pursuit = std::make_shared<PurePursuit>();
  rclcpp::spin(pure_pursuit);
  rclcpp::shutdown();
  return 0;
}
