#include <ping360_sonar/sonar_interface.h>
#include <thread>


constexpr static int firmwareMaxSamples{1200};
constexpr static int firmwareMinSamplePeriod{80};
constexpr static float samplePeriodTickDuration{25e-9f};
constexpr static ushort firmwareMinTransmitDuration{5};
constexpr static ushort firmwareMaxTransmitDuration{500};
constexpr static float maxDurationRatio{64e6};

using namespace ping360_sonar;


Ping360Interface::Ping360Interface(std::string device, int baudrate, bool fallback, std::string connection_type, std::string udp_address, int udp_port)
{
  if(connection_type.compare("serial") == 0)
  {
    serial_link = std::make_unique<SerialLink>(device, baudrate);
    sonar = std::make_unique<Ping360>(*serial_link.get());
  }
  else if(connection_type.compare("udp") ==0)
  {
    udp_link = std::make_unique<UdpLink>(udp_address, std::to_string(udp_port));
    sonar = std::make_unique<Ping360>(*udp_link.get());
  }

  // try to init the real sonar anyway
  if(sonar->initialize())
  {
    real_sonar = true;
    return;
  }
  if(!fallback)
    throw std::runtime_error("Cannot initialize sonar");

  real_sonar = false;
}

std::pair<int, int> Ping360Interface::configureAngles(int aperture_deg, int step_deg, bool ensure_divisor)
{
  // to gradians
  const auto target_half_aperture{int(aperture_deg*200./360+0.5)};
  auto best_half_aperture{target_half_aperture};
  angle_step = round(step_deg*400./360);

  // ensure angle_step is a divisor of max-min in gradians, necessary for LaserScan messages
  if(ensure_divisor)
  {
    // look around step, allow increased aperture
    const auto target_step{angle_step};

    // not too far from requested aperture, as close as possible to requested step (impacts turn duration)
    const auto computeCost = [&](int step, int half_aperture)
    {
      if(half_aperture % step != 0)
        return 1000;
      return abs(step-target_step) + abs(half_aperture-target_half_aperture);
    };

    auto best_cost{computeCost(angle_step, target_half_aperture)};
    if(best_cost != 0)
    {
      for(int step = 1; step < target_step*2; ++step)
      {
        for(int half_aperture = target_half_aperture;
            half_aperture <= std::min(target_half_aperture+10, 200); half_aperture++)
        {
          if(const auto cost{computeCost(step, half_aperture)}; cost < best_cost)
          {
            angle_step = step;
            best_cost = cost;
            best_half_aperture = half_aperture;
          }
        }
      }
    }
  }

  angle_min = -best_half_aperture;
  angle_max = best_half_aperture;
  if(fullScan())
    angle_max -= angle_step;

  // check current angle wrt. new config
  if(angle < angle_min || angle > angle_max || (angle-angle_min) % angle_step != 0)
    angle = 0;

  // message of actual angle configuration
  return {round(best_half_aperture*360./200), round(angle_step*360./400)};
}

void Ping360Interface::configureTransducer(uint8_t gain, uint16_t frequency, uint16_t speed_of_sound, float range)
{
  max_range = range;
  auto &device{sonar->device_data_data};
  device.mode = 1;
  device.gain_setting = gain;
  device.transmit_frequency = frequency;

  // find maximum possible samples for this range
  device.number_of_samples = std::min<uint16_t>(firmwareMaxSamples,
                                                2.f*range/(firmwareMinSamplePeriod * speed_of_sound * samplePeriodTickDuration));
  device.sample_period = 2.f*range / (device.number_of_samples * speed_of_sound * samplePeriodTickDuration);

  // transmit duration depends on max range + hardware limits
  /*
   * Per firmware engineer:
   * 1. Starting point is TxPulse in usec = ((one-way range in metres) * 8000) / (Velocity of sound in metres
   * per second)
   * 2. Then check that TxPulse is wide enough for currently selected sample interval in usec, i.e.,
   *    if TxPulse < (2.5 * sample interval) then TxPulse = (2.5 * sample interval)
   * 3. Perform limit checking
  */
  // 1
  const auto one_way_duration_us{std::round((8000.f*range)/speed_of_sound)};
  // 2 (transmit duration is microseconds, sample_period_ns is nanoseconds)
  const auto sample_period_ns{device.sample_period * samplePeriodTickDuration};
  device.transmit_duration = std::max<ushort>(2.5f*sample_period_ns/1000, one_way_duration_us);
  // 3 ensure bounds
  if(device.transmit_duration < firmwareMinTransmitDuration)
    device.transmit_duration = firmwareMinTransmitDuration;
  else
  {
    const auto max_duration{std::min<ushort>(firmwareMaxTransmitDuration, sample_period_ns*maxDurationRatio)};
    if(device.transmit_duration > max_duration)
      device.transmit_duration = max_duration;
  }

  if(!real_sonar)
  {
    const auto samples{device.number_of_samples};
    if(device.data != nullptr && device.data_length != samples)
      delete[] device.data;

    device.data_length = samples;

    if(device.data == nullptr)
      device.data = new uint8_t[samples];
  }
}

bool Ping360Interface::updateAngle()
{
  angle += angle_step;
  if(fullScan())
  {
    const auto end_turn{angle + angle_step > angle_max};
    if(angle > angle_max)
      angle = angle_min;
    return end_turn;
  }

  // sector scan, check near end of sector
  if(angle + angle_step >= angle_max || angle + angle_step <= angle_min)
  {
    angle_step *= -1;
    return true;
  }
  return false;
}

std::pair<bool, bool> Ping360Interface::read()
{
  // update angle before ping in order to stay sync
  const auto end_turn = updateAngle();

  auto &device{sonar->device_data_data};

  if(real_sonar)
  {
    std::cout << device.transmit_duration << std::endl;
    sonar->set_transducer(device.mode,
                         device.gain_setting,
                         angle > 0 ? angle : angle+400,
                         device.transmit_duration,
                         device.sample_period,
                         device.transmit_frequency,
                         device.number_of_samples,
                         1,
                         0);
    return {sonar->waitMessage(Ping360Id::DEVICE_DATA, timeout) != nullptr, end_turn};
  }

  // emulated sonar: randomly populate data
  const auto length{samples()};
  const auto scale{5*abs((angle+400) % 400 - 200)};
  for(int i = 0; i < length; ++i)
  {
    if(rand() % length + length < 1.1*i + scale)
      device.data[i] = 120 + rand() % 120;
    else
      device.data[i] = 0;
  }
  // simulate transmit duration
  std::this_thread::sleep_for(std::chrono::microseconds(device.transmit_duration));
  return {true, end_turn};
}
