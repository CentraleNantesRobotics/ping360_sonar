#ifndef PING360_SONAR_NODE_H
#define PING360_SONAR_NODE_H

// basic node to begin the project

#include <rclcpp/rclcpp.hpp>
#include <ping360_sonar/ping360.h>

using namespace std::chrono_literals;

namespace ping360_sonar
{

class Ping360Sonar : public rclcpp::Node
{
public:
  Ping360Sonar(rclcpp::NodeOptions options = rclcpp::NodeOptions());


private:

  // sonar i/o
  Ping360 sonar;

  // ROS i/o

};
}

#endif
