#include <altimeter_ping360/altimeter.hpp>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ping360_sonar::Altimeter>());
  rclcpp::shutdown();
  return 0;
}

