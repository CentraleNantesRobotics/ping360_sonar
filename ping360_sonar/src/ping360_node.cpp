// basic node to begin the project

#include <ping360_sonar/ping360_node.h>

using namespace std::chrono_literals;
using namespace ping360_sonar;



Ping360Sonar::Ping360Sonar(rclcpp::NodeOptions options)
  : Node("ping360", options),
    link(declare_parameter<std::string>("port", "/dev/ttyUSB0"),
         declare_parameter<int>("baudrate", 115200U)),
    sonar(link)
{ 




}

