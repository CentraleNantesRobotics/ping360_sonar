#ifndef PING360_SONAR_INTERFACE
#define PING360_SONAR_INTERFACE

#include <device/ping-device-ping360.h>
#include <message/ping-message-ping360.h>
#include <hal/link/desktop/serial-link.h>

namespace ping360_sonar
{

class Ping360Interface
{
public:
  Ping360Interface(std::string device, int baudrate, bool real_sonar)
    : serial_link(device, baudrate), sonar(serial_link), real_sonar{real_sonar}
  {}
  void initialize();
  ~Ping360Interface()
  {
    if(!real_sonar)
      return;
    sonar.set_motor_off();
    sonar.waitMessage(CommonId::ACK, 1000);
  }
  std::pair<bool, bool> read();

  std::string configureAngles(int min, int max, int step);
  void configureTransducer(uint8_t gain, uint16_t samples, uint16_t frequency, uint16_t speed_of_sound, float range);

  inline float rangeFrom(int index) const
  {
    return (index+1)*max_range/samples();
  }
  inline float minAngle() const {return grad2rad(min_angle);}
  inline float maxAngle() const {return grad2rad(max_angle);}
  inline float angleStep() const {return grad2rad(step_angle);}
  inline float currentAngle() const {return grad2rad(angle);}
  inline size_t angleCount() const {return (max_angle-min_angle-1)/step_angle;}
  inline size_t angleIndex() const {return (angle-min_angle)/step_angle;}

  inline uint16_t samples() const
  {
    return sonar.device_data_data.number_of_samples;
  }
  inline std::pair<const uint8_t*, uint16_t> intensities() const
  {
    return {sonar.device_data_data.data, sonar.device_data_data.data_length};
  }
  inline double transmitDuration() const
  {
    // micro to seconds
    return sonar.device_data_data.transmit_duration/1e6;
  }

private:
  SerialLink serial_link;
  Ping360 sonar;
  bool real_sonar{false};
  float max_range{};

  // angular params
  int min_angle{}, max_angle{}, step_angle{};
  int angle{};

  static inline float grad2rad(int grad)
  {
    return (2*M_PI*grad)/400;
  }
};


}


#endif
