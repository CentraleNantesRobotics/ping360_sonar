
#include <ping360_sonar/ping360_node.h>
#include <ping360_sonar/sector.h>
#include <ping-message-common.h>
#include <ping-message-ping360.h>

using namespace std::chrono_literals;
using namespace ping360_sonar;
using std::string;
using std::vector;

Ping360Sonar::Ping360Sonar(rclcpp::NodeOptions options)
  : Node("ping360", options)
{ 
  // bounded parameters that are parsed later
  declareParamDescription("gain", 0, "Sonar gain (0 = low, 1 = normal, 2 = high)", 0, 2);
  declareParamDescription("frequency", 740, "Sonar operating frequency [kHz]", 650, 850);
  declareParamDescription("range_max", 2, "Sonar max range [m]", 1, 50);
  declareParamDescription("angle_sector", 360, "Scanned angular sector around sonar heading [degrees]. Will oscillate if not 360", 60, 360);
  declareParamDescription("angle_step", 1, "Sonar angular resolution [degrees]", 1, 20);
  declareParamDescription("image_size", 300, "Output image size [pixels]", 100, 1000, 2);
  declareParamDescription("scan_threshold", 200, "Intensity threshold for LaserScan message", 1, 255);
  declareParamDescription("speed_of_sound", 1500, "Speed of sound [m/s]", 1450, 1550);
  declareParamDescription("image_rate", 100, "Image publishing rate [ms]", 50, 2000);

  // other, unbounded params
  publish_image = declareParamDescription("publish_image", true, "Publish images on 'scan_image'");
  publish_scan = declareParamDescription("publish_scan", false, "Publish laserscans on 'scan'");
  publish_echo = declareParamDescription("publish_echo", false, "Publish raw echo on 'scan_echo'");

  // constant initialization
  const auto frame{declareParamDescription<string>("frame", "sonar", "Frame ID of the message headers")};
  image.header.set__frame_id(frame);
  image.set__encoding("mono8");
  image.set__is_bigendian(0);
  scan.header.set__frame_id(frame);
  scan.set__range_min(0.75);
  echo.header.set__frame_id(frame);

  // ROS interface
  configureFromParams();

  const auto image_rate_ms{get_parameter("image_rate").as_int()};
  image_timer = this->create_wall_timer(std::chrono::milliseconds(image_rate_ms),
                                        [this](){publishImage();});

  param_change = add_on_set_parameters_callback(
                   std::bind(&Ping360Sonar::parametersCallback, this, std::placeholders::_1));
}

Ping360Sonar::IntParams Ping360Sonar::updatedParams(const std::vector<rclcpp::Parameter> &new_params) const
{
  // "only" parameters to be monitored for change
  using ParamType = rclcpp::ParameterType;
  const std::map<ParamType,vector<string>> mutable_params{
    {ParamType::PARAMETER_INTEGER,{"gain","frequency","range_max",
                                   "angle_sector","angle_step",
                                   "speed_of_sound","image_size", "scan_threshold"}},
    {ParamType::PARAMETER_BOOL, {"publish_image","publish_scan","publish_echo"}}};

  IntParams mapping;
  for(const auto &[type,names]: mutable_params)
  {
    const auto params{get_parameters(names)};
    if(type == ParamType::PARAMETER_INTEGER)
    {
      for(auto &param: params)
        mapping[param.get_name()] = param.as_int();
    }
    else
    {
      for(auto &param: params)
        mapping[param.get_name()] = param.as_bool();
    }
  }
  // override with new ones
  for(auto &param: new_params)
  {
    if(param.get_type() == ParamType::PARAMETER_BOOL)
      mapping[param.get_name()] = param.as_bool();
    else if(param.get_type() == ParamType::PARAMETER_INTEGER)
      mapping[param.get_name()] = param.as_int();
  }

  return mapping;
}

SetParametersResult Ping360Sonar::parametersCallback(const vector<rclcpp::Parameter> &parameters)
{
  configureFromParams(parameters);
  return SetParametersResult().set__successful(true);
}

void Ping360Sonar::initPublishers(bool image, bool scan, bool echo)
{
#ifdef PING360_PUBLISH_RELIABLE
  const auto qos{rclcpp::QoS(5)};
#else
  const auto qos{rclcpp::SensorDataQoS()};
#endif

  publish_echo = echo;
  publish_image = image;
  publish_scan = scan;

  if(publish_image && image_pub.getTopic().empty())
    image_pub = image_transport::create_publisher(this, "scan_image");

  if(publish_echo && echo_pub == nullptr)
    echo_pub = create_publisher<ping360_sonar_msgs::msg::SonarEcho>("scan_echo", qos);

  if(publish_scan && scan_pub == nullptr)
    scan_pub = create_publisher<sensor_msgs::msg::LaserScan>("scan", qos);
}

void Ping360Sonar::configureFromParams(const vector<rclcpp::Parameter> &new_params)
{
  // get current params updated with new ones, if any
  const auto params{updatedParams(new_params)};

  // forward to configuration
  const auto [angle_sector, step] = sonar.configureAngles(params.at("angle_sector"),
      params.at("angle_step"),
      params.at("publish_scan")); {}
  // inform if requested angle config cannot be met because of gradians
  if(angle_sector != params.at("angle_sector") || step != params.at("angle_step"))
  {
    RCLCPP_INFO(get_logger(),
                "Due to sonar using gradians, sector is %i (requested %i) and step is %i (requested %i)",
                angle_sector, params.at("angle_sector"), step, params.at("angle_step"));
  }

  initPublishers(params.at("publish_image"),
                 params.at("publish_scan"),
                 params.at("publish_echo"));

  sonar.configureTransducer(params.at("gain"),
                            params.at("frequency"),
                            params.at("speed_of_sound"),
                            params.at("range_max"));

  // forward to message meta-data
  echo.set__gain(params.at("gain"));
  echo.set__range(params.at("range_max"));
  echo.set__speed_of_sound(params.at("speed_of_sound"));
  echo.set__number_of_samples(sonar.samples());
  echo.set__transmit_frequency(params.at("frequency"));

  scan.set__range_max(params.at("range_max"));
  scan.set__time_increment(sonar.transmitDuration());
  scan.set__angle_max(sonar.angleMax());
  scan.set__angle_min(sonar.angleMin());
  scan.set__angle_increment(sonar.angleStep());

  const int size{params.at("image_size")};
  if(size != static_cast<int>(image.step) ||
     std::any_of(new_params.begin(), new_params.end(),
                 [](const auto &param){return param.get_name() == "angle_sector";}))
  {
    image.data.resize(size*size);
    std::fill(image.data.begin(), image.data.end(), 0);
    image.height = image.width = image.step = size;
  }

  sector.configure(sonar.samples(), size/2);
  scan_threshold = params.at("scan_threshold");
}


void Ping360Sonar::publishEcho(const rclcpp::Time &now)
{
  const auto [data, length] = sonar.intensities(); {}
  echo.angle = sonar.currentAngle();
  echo.intensities.resize(length);
  std::copy(data, data+length, echo.intensities.begin());
  echo.header.set__stamp(now);
  echo_pub->publish(echo);
}

void Ping360Sonar::publishScan(const rclcpp::Time &now, bool end_turn)
{
  // write latest reading
  scan.ranges.resize(sonar.angleCount());
  scan.intensities.resize(sonar.angleCount());

  const auto angle{sonar.angleIndex()};
  auto &this_range = scan.ranges[angle] = 0;
  auto &this_intensity = scan.intensities[angle] = 0;

  // find first (nearest) valid point in this direction
  const auto [data, length] = sonar.intensities(); {}
  for(int index=0; index<length; index++)
  {
    if(data[index] >= scan_threshold)
    {
      if(const auto range{sonar.rangeFrom(index)};
         range >= scan.range_min && range < scan.range_max)
      {
        this_range = range;
        this_intensity = data[index]/255.f;
        break;
      }
    }
  }

  if(end_turn)
  {
    if(!sonar.fullScan())
    {
      if(sonar.angleStep() < 0)
      {
        // now going negative: scan was positive
        scan.set__angle_max(sonar.angleMax());
        scan.set__angle_min(sonar.angleMin());
      }
      else
      {
        // now going positive: scan was negative
        scan.set__angle_max(sonar.angleMin());
        scan.set__angle_min(sonar.angleMax());
      }
      scan.set__angle_increment(-sonar.angleStep());
      scan.angle_max -= scan.angle_increment;
    }
    scan.header.set__stamp(now);
    scan_pub->publish(scan);
  }
}

void Ping360Sonar::refreshImage()
{
  const auto [data, length] = sonar.intensities(); {}
  if(length == 0) return;
  const auto half_size{image.step/2};

  sector.init(sonar.currentAngle(), fabs(sonar.angleStep()));
  int x{}, y{}, index{};

  while(sector.nextPoint(x, y, index))
  {
    if(index < length)
      image.data[half_size-y + image.step*(half_size-x)] = data[index];
  }
}

void Ping360Sonar::refresh()
{
  const auto &[valid, end_turn] = sonar.read(); {}
  
  if(!valid)
  {
    RCLCPP_WARN(get_logger(), "Cannot communicate with sonar");
    return;
  }

  const auto now{this->now()};
  if(publish_echo && echo_pub->get_subscription_count())
    publishEcho(now);

  if(publish_image)
    refreshImage();

  if(publish_scan && scan_pub->get_subscription_count())
    publishScan(now, end_turn);
}

void Ping360Sonar::publishImage()
{
  if(publish_image)
  {
    image.header.set__stamp(now());
    image_pub.publish(image);
  }
}
