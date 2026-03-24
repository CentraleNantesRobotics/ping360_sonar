
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
  declareParamDescription("sonar_timeout", 8000, "Sonar timeout [ms]", 0, 20000);

  // other, unbounded params
  publish_image = declareParamDescription("publish_image", true, "Publish images on 'scan_image'");
  publish_scan = declareParamDescription("publish_scan", false, "Publish laserscans on 'scan'");
  publish_echo = declareParamDescription("publish_echo", false, "Publish raw echo on 'scan_echo'");
  publish_distance = declareParamDescription("publish_distance", true, "Publish estimated distance on 'estimated_distance'");

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
                                   "speed_of_sound","image_size", "scan_threshold", "sonar_timeout"}},
    {ParamType::PARAMETER_BOOL, {"publish_image","publish_scan","publish_echo", "publish_distance"}}};

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

void Ping360Sonar::initPublishers(bool image, bool scan, bool echo, bool distance)
{
#ifdef PING360_PUBLISH_RELIABLE
  const auto qos{rclcpp::QoS(5)};
#else
  const auto qos{rclcpp::SensorDataQoS()};
#endif

  publish_echo = echo;
  publish_image = image;
  publish_scan = scan;
  publish_distance = distance;

  if(publish_image && image_pub.getTopic().empty())
    image_pub = image_transport::create_publisher(this, "scan_image");

  if(publish_echo && echo_pub == nullptr)
    echo_pub = create_publisher<ping360_sonar_msgs::msg::SonarEcho>("scan_echo", qos);

  if(publish_scan && scan_pub == nullptr)
    scan_pub = create_publisher<sensor_msgs::msg::LaserScan>("scan", qos);

  if(publish_distance && distance_pub == nullptr)
    distance_pub = create_publisher<std_msgs::msg::Float32>("estimated_distance", qos);
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
                 params.at("publish_echo"),
                 params.at("publish_distance"));

  sonar.configureTransducer(params.at("gain"),
                            params.at("frequency"),
                            params.at("speed_of_sound"),
                            params.at("range_max"));
  sonar.setTimeout(params.at("sonar_timeout"));

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

  // auto eigenDataVec = Eigen::Map<const Eigen::Vector<uint8_t, Eigen::Dynamic>>(data, length);

  // Pass a Laplacian of Gaussians along the ray
  // std::vector<double> kernel = {-1.0, -3.0, 8.0, 8.0, -3.0, -1.0};
  // std::vector<uint8_t> filtered = convolveLoG({data, length}, kernel);
  // std::cout << "Input data: \n" 
  //           << static_cast<int>(data[0]) << " " << static_cast<int>(data[1]) << " " << static_cast<int>(data[2]) << "\n"
  //           << "Filtered data: \n"
  //           << static_cast<int>(filtered[0]) << " " << static_cast<int>(filtered[1]) << " " << static_cast<int>(filtered[2])
  //           << std::endl;

  // for (size_t i = 0; i < filtered.size(); ++i) {  
  //   if (i < 80) {
  //     filtered[i] = 0;
  //   }
  // }

  // std::cout << find_index(filtered) << std::endl;

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

  if(publish_distance && distance_pub->get_subscription_count())
    publishDistance(end_turn);
}

void Ping360Sonar::publishImage()
{
  if(publish_image)
  {
    image.header.set__stamp(now());
    image_pub.publish(image);
  }
}

void Ping360Sonar::publishDistance(bool end_turn)
{
  if(publish_distance)
  {
    constexpr float min_range = 0.2f;
    const auto [data, length] = sonar.intensities(); {}

    // Pass a Laplacian of Gaussians along the ray
    std::vector<double> kernel = {-1.0, -3.0, 8.0, 8.0, -3.0, -1.0};
    std::vector<uint8_t> filtered = convolveLoG({data, length}, kernel);

    // Threshold and min index 
    for (size_t i = 0; i < filtered.size(); ++i) {  
      if (i < 80) {
        filtered[i] = 0;
      }
    }

    dist_index_buffer.push_back(find_index(filtered));
  }

    if(end_turn)
    {
      // Find first quartile
      std::sort(dist_index_buffer.begin(), dist_index_buffer.end());
      int index_quartile = dist_index_buffer.size() / 4;

      // Publish distance data
      distance.data = sonar.rangeFrom(dist_index_buffer[index_quartile]);
      distance_pub->publish(distance);
      
      // Clear buffer
      dist_index_buffer.clear();
    }
  
}

// Convolve uint8 buffer with kernel
std::vector<uint8_t> Ping360Sonar::convolveLoG(const std::pair<const uint8_t*, uint16_t>& data,
                                               const std::vector<double>& kernel)
{
  const uint8_t* input = data.first;
    uint16_t length = data.second;

    int ksize = kernel.size();
    int half = ksize / 2;

    std::vector<double> temp(length, 0.0);

    // Step 1: Convolution (same as before)
    for (uint16_t i = 0; i < length; ++i)
    {
        double sum = 0.0;

        for (int k = -half; k <= half; ++k)
        {
            int idx = static_cast<int>(i) + k;

            // Clamp boundaries
            if (idx < 0) idx = 0;
            if (idx >= length) idx = length - 1;

            sum += kernel[k + half] * static_cast<double>(input[idx]);
        }

        temp[i] = sum;
    }

    // Step 2: Find min and max
    double min_val = temp[0];
    double max_val = temp[0];

    for (double v : temp)
    {
        if (v < min_val) min_val = v;
        if (v > max_val) max_val = v;
    }

    // Avoid division by zero
    double range = max_val - min_val;
    if (range == 0.0)
        range = 1.0;

    // Step 3: Normalize to [0, 255]
    std::vector<uint8_t> output(length);

    for (uint16_t i = 0; i < length; ++i)
    {
        double normalized = (temp[i] - min_val) / range;  // [0,1]
        output[i] = static_cast<uint8_t>(normalized * 255.0);
    }

    return output;
}

int Ping360Sonar::find_index(const std::vector<uint8_t>& v) {
    auto it = std::find_if(v.begin(), v.end(), [](uint8_t x) {
        return x >= 130;
    });

    if (it != v.end()) {
        return std::distance(v.begin(), it);
    }
    return v.size(); // not found
}
