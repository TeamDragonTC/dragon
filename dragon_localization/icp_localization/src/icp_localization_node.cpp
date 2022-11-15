#include <icp_localization/icp_localization.hpp>

int main(int argc, char** argv)
{
	rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<ICPLocalization>());

  rclcpp::shutdown();

	return 0;
}
