#include <altimeter_ping360/altimeter.hpp>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;

  // Allow service changing logging verbosity
  options.enable_logger_service(true);  

  // Spinning the node
  rclcpp::spin(std::make_shared<ping360_sonar::Altimeter>(options));

  rclcpp::shutdown();
  return 0;
}

