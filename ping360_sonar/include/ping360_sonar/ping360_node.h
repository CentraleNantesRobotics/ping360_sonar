#ifndef PING360_SONAR_NODE_H
#define PING360_SONAR_NODE_H

// basic node to begin the project

#include <rclcpp/rclcpp.hpp>
#include <device/ping-device-ping360.h>
#include <hal/link/desktop/serial-link.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <ping360_sonar_msgs/msg/sonar_echo.hpp>
#include <opencv2/core/mat.hpp>
#include <cv_bridge/cv_bridge.h>

using namespace std::chrono_literals;

namespace ping360_sonar
{

class Ping360Sonar : public rclcpp::Node
{
public:
  Ping360Sonar(rclcpp::NodeOptions options = rclcpp::NodeOptions());


private:

  // ROS i/o

  void getSonarData();

  float calculateRange(int, float _samplePeriodTickDuration=25e-9);

  int calculateSamplePeriod(float _samplePeriodTickDuration=25e-9);

  int adjustTransmitDuration(int _firmwareMinTransmitDuration=5);

  int transmitDurationMax(int _firmwareMaxTransmitDuration=500);

  int getSamplePeriod(float _samplePeriodTickDuration = 25e-9);

  void updateSonarConfig();

  void dataSelection();

  sensor_msgs::msg::Image generateImageMsg();

  sensor_msgs::msg::LaserScan generateScanMsg();

  ping360_sonar_msgs::msg::SonarEcho generateRawMsg();

  void updateAngle();

  ping_message* transmitAngle(int);

  ping_message* set_mode(int);

  ping_message* set_angle(int);

  ping_message* set_gain_setting(int);

  ping_message* set_transmit_duration(int);

  ping_message* set_transmit_frequency(int);

  ping_message* set_sample_period(int);

  ping_message* set_number_of_samples(int);

  void timerCallback();


private:

  //get integers
  int _baudrate = declare_parameter<int>("baudrate", 115200);
  int _gain = declare_parameter<int>("gain", 0);
  int _transmit_frequency = declare_parameter<int>("transmit_frequency", 740);
  int _queue_size = declare_parameter<int>("queue_size", 1);
  int _max_angle = declare_parameter<int>("maxAngle", 400);
  int _min_angle = declare_parameter<int>("minAngle", 0);
  int _threshold = declare_parameter<int>("threshold", 200);
  int _number_of_samples = declare_parameter<int>("numberOfSamples", 200);
  int _step = declare_parameter<int>("step", 1);
  int _img_size = declare_parameter<int>("imgSize", 500);
  int _speed_of_sound = declare_parameter<int>("speedOfSound", 1500);
  int _sonar_range = declare_parameter<int>("sonarRange", 1);

  //get doubles


  //get strings
  std::string _device = declare_parameter<std::string>("device", "/dev/ttyUSB0");
  std::string _frame_id = declare_parameter<std::string>("frameID", "sonar_frame");

  //get booleens
  bool _debug = declare_parameter<bool>("debug", false);
  bool _enable_image_topic = declare_parameter<int>("enableImageTopic", true);
  bool _enable_scan_topic = declare_parameter<bool>("enableScanTopic", true);
  bool _enable_data_topic = declare_parameter<bool>("enableDataTopic", true);
  bool _oscillate = declare_parameter<bool>("oscillate", true);

  //declare and compute other variables
  bool _updated = true;
  int _angle = _min_angle;
  int _sign = 1;
  int _transmit_duration = adjustTransmitDuration();
  int _sample_period = getSamplePeriod();

  std::vector<int> _data;
  std::vector<uint8_t> _raw_data;
  std::vector<float> _ranges;
  std::vector<float> _intensities;

  cv::Point _center = cv::Point(float(_img_size/2), float(_img_size/2));
  cv_bridge::CvImage _cv_bridge;

  // sonar i/o
  SerialLink _link = SerialLink(_device, _baudrate);
  Ping360 _sensor = Ping360(_link);

  // ROS i/o
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr _image_publisher;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr _scan_publisher;
  rclcpp::Publisher<ping360_sonar_msgs::msg::SonarEcho>::SharedPtr _data_publisher;

};
}

#endif
