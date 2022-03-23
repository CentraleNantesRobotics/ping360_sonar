#include <ping360_sonar/ping360_node.h>
#include <rclcpp/rclcpp.hpp>

// boilerplate main
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ping360_sonar::Ping360Sonar>());
  rclcpp::shutdown();
  return 0;
}
