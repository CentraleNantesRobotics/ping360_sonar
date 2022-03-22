// basic node to begin the project

#include <ping360_sonar/ping360_node.h>

using namespace std::chrono_literals;

namespace ping360_sonar
{

Ping360Sonar::Ping360Sonar(rclcpp::NodeOptions options) : Node("ping360", options)
{




}


}

// boilerplate main
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ping360_sonar::Ping360Sonar>());
  rclcpp::shutdown();
  return 0;
}
