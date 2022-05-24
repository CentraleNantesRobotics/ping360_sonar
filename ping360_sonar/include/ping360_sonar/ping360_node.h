#ifndef PING360_SONAR_NODE_H
#define PING360_SONAR_NODE_H

#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <ping360_sonar_msgs/msg/sonar_echo.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>

#include <ping360_sonar/sector.h>
#include <ping360_sonar/sonar_interface.h>

using namespace std::chrono_literals;
using ping360_sonar_msgs::msg::SonarEcho;
using sensor_msgs::msg::LaserScan;
using sensor_msgs::msg::Image;
using rcl_interfaces::msg::SetParametersResult;

namespace ping360_sonar
{

class Ping360Sonar : public rclcpp::Node
{
  using IntParams = std::map<std::string, int>;

public:
  Ping360Sonar(rclcpp::NodeOptions options = rclcpp::NodeOptions());
  void refresh();

private:

  rclcpp::TimerBase::SharedPtr image_timer;
  OnSetParametersCallbackHandle::SharedPtr param_change;
  SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter> &parameters);
  IntParams updatedParams(const std::vector<rclcpp::Parameter> &new_params) const;
  void configureFromParams(const std::vector<rclcpp::Parameter> &new_params = {});

  // helper functions to declare and describe parameters
  template <typename ParamType>
  inline ParamType declareParamDescription(std::string name,
                                           ParamType default_value,
                                           std::string description)
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.set__name(name).set__description(description);
    return declare_parameter<ParamType>(name, default_value, descriptor);
  }
  inline int declareParamDescription(std::string name,
                                     int default_value,
                                     std::string description,
                                     int lower,
                                     int upper,
                                     int step = 1)
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.set__name(name).set__description(description);
    descriptor.integer_range = {rcl_interfaces::msg::IntegerRange()
                                .set__from_value(lower)
                                .set__to_value(upper)
                                .set__step(step)};
    return declare_parameter<int>(name, default_value, descriptor);
  }

  // sonar i/o
  Ping360Interface sonar{declare_parameter<std::string>("device", "/dev/ttyUSB0"),
        static_cast<int>(declare_parameter<int>("baudrate", 115200)),
        declareParamDescription("fallback_emulated", true, "Emulates a sonar if Ping360 cannot be initialized"),
        declareParamDescription<std::string>("connection_type", "serial", "If connection is via serial or udp"),
        declareParamDescription<std::string>("udp_address", "0.0.0.0", "Udp address"),
        declareParamDescription<int>("udp_port", 12345, "Udp port")};
  inline void initPublishers(bool image, bool scan, bool echo);

  // image params
  bool publish_image{};
  Sector sector;
  image_transport::Publisher image_pub;
  sensor_msgs::msg::Image image;
  void configureMessageFomParams();
  void refreshImage();
  inline void publishImage();

  // laserscan
  bool publish_scan{};
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub;
  LaserScan scan;
  int scan_threshold{};
  void publishScan(const rclcpp::Time &now, bool end_turn);

  // raw echo
  bool publish_echo{};
  rclcpp::Publisher<ping360_sonar_msgs::msg::SonarEcho>::SharedPtr echo_pub;
  SonarEcho echo;
  void publishEcho(const rclcpp::Time &now);
};
}

#endif
