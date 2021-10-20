// basic node to begin the project

#include <ping360_sonar/ping360_node.h>

using namespace std::chrono_literals;

namespace ping360_sonar
{

Ping360Sonar::Ping360Sonar(rclcpp::NodeOptions options) : Node("ping360", options)
{




}




}



#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(ping360_sonar::Ping360Sonar)
