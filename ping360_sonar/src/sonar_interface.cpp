#include <ping360_sonar/sonar_interface.h>

constexpr static float samplePeriodTickDuration{25e-9f};
constexpr static int firmwareMinTransmitDuration{5};
constexpr static float firmwareMaxTransmitDuration{500};
constexpr static float maxDurationRatio{64e6};

using namespace ping360_sonar;

void Ping360Interface::initialize()
{
#ifdef REAL_SONAR
  if(!sonar.initialize())
  {
    throw std::runtime_error("Cannot initialize sonar");
  }
#endif
}

std::string Ping360Interface::configureAngles(int min, int max, int step)
{
  if(min_angle == min && max_angle == max && step_angle == step)
    return {};

  if(max <= min || (max - min) % step != 0)
  {
    return "inconsistent angular settings: "
           " angular range is [" + std::to_string(min)
        + " - " + std::to_string(max) + "] while step is " + std::to_string(step);
  }

  angle = min;
  min_angle = min;
  max_angle = max;
  step_angle = step;
  return {};
}

void Ping360Interface::configureTransducer(uint8_t gain, uint16_t samples, uint16_t frequency, uint16_t speed_of_sound, float range)
{
  max_range = range;
  auto &device{sonar.device_data_data};
  device.mode = 1;
  device.gain_setting = gain;
  device.number_of_samples = samples;
  device.transmit_frequency = frequency;

  // sample period [unit-less] depends on number of samples and max range
  device.sample_period = int((2.*range)/(samples*speed_of_sound*samplePeriodTickDuration));

  // transmit duration depends on max range + hardware limits
  // TODO computation is not correct, outputs 0
  // sample period in ms
  const auto sample_period_ms{(2.f*range)/(samples*speed_of_sound*1000)};
  const auto one_way_duration{(8000.f*range)/speed_of_sound};
  const auto target_duration = std::max(2.5f*sample_period_ms, one_way_duration);
  const auto max_duration{std::min(firmwareMaxTransmitDuration, sample_period_ms*maxDurationRatio)};
  device.transmit_duration = target_duration;
  if(target_duration > max_duration)
    device.transmit_duration = max_duration;
  if(target_duration < firmwareMinTransmitDuration)
    device.transmit_duration = firmwareMinTransmitDuration;
  //device.transmit_duration = std::clamp<int>(target_duration, firmwareMinTransmitDuration, max_duration);

#ifndef REAL_SONAR  
  if(device.data != nullptr && device.data_length != samples)
      delete[] device.data;

  device.data_length = samples;

  if(device.data == nullptr)
    device.data = new uint8_t[samples];
#endif
}

std::pair<bool, bool> Ping360Interface::read()
{
  // update angle before ping in order to stay sync
  angle += step_angle;
  const auto end_turn{angle + step_angle == max_angle};
  if(angle == max_angle)
    angle = min_angle;

  auto &device{sonar.device_data_data};
#ifdef REAL_SONAR
  sonar.set_transducer(device.mode,
                       device.gain_setting,
                       angle,
                       device.transmit_duration,
                       device.sample_period,
                       device.transmit_frequency,
                       device.number_of_samples,
                       1,
                       0);
  return {sonar.waitMessage(Ping360Id::DEVICE_DATA, 8000) != nullptr, end_turn};
#else
  // randomly populate data around every pi/2
  const auto length{samples()};
  if(angle % 100 <  100)
  {
    for(int i = 0; i < length; ++i)
      device.data[i] = 120 + rand() % 120;
  }
  else
  {
    for(int i = 0; i < length; ++i)
      device.data[i] = 0;
  }
  // simulate transmit duration
  std::this_thread::sleep_for(std::chrono::microseconds(device.transmit_duration));
  return {true, end_turn};
#endif
}
