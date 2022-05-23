#ifndef PING360_SONAR_INTERFACE
#define PING360_SONAR_INTERFACE

#include <device/ping-device-ping360.h>
#include <message/ping-message-ping360.h>
#include <hal/link/desktop/serial-link.h>
#include <hal/link/desktop/udp-link.h>

namespace ping360_sonar
{

class Ping360Interface
{
public:
  Ping360Interface(std::string device, int baudrate, bool fallback, std::string connection_type, std::string udp_address, int udp_port);
  ~Ping360Interface()
  {
    if(!real_sonar)
      return;
    sonar->set_motor_off();
    sonar->waitMessage(CommonId::ACK, 1000);
  }
  std::pair<bool, bool> read();

  std::pair<int, int> configureAngles(int aperture_deg, int step_deg, bool align_step);
  void configureTransducer(uint8_t gain, uint16_t frequency, uint16_t speed_of_sound, float range);

  inline float rangeFrom(int index) const
  {
    return (index+1)*max_range/samples();
  }
  inline bool fullScan() const {return angle_min == -200;}
  inline float angleMin() const {return grad2rad(angle_min);}
  inline float angleMax() const {return grad2rad(angle_max);}
  inline float angleStep() const {return grad2rad(angle_step);}
  inline float currentAngle() const {return grad2rad(angle);}
  inline size_t angleCount() const {return (angle_max-angle_min)/abs(angle_step);}
  inline size_t angleIndex() const
  {
    if(angle_step > 0)
      return (angle-angle_min)/angle_step;
    return (angle-angle_max)/angle_step;
  }
  bool updateAngle();

  void setTimeout(int newTimeout){timeout = newTimeout;}
  int getTimeout(){return timeout;}

  inline uint16_t samples() const
  {
    return sonar->device_data_data.number_of_samples;
  }
  inline std::pair<const uint8_t*, uint16_t> intensities() const
  {
    return {sonar->device_data_data.data, sonar->device_data_data.data_length};
  }
  inline double transmitDuration() const
  {
    // micro to seconds
    return sonar->device_data_data.transmit_duration/1e6;
  }

private:
  std::unique_ptr<SerialLink> serial_link;
  std::unique_ptr<UdpLink> udp_link;
  std::unique_ptr<Ping360> sonar;
  bool real_sonar{false};
  float max_range{};

  // angular params
  bool oscillate;
  int angle_min{}, angle_max{}, angle_step{};
  int angle{};
  int timeout{8000};


  static inline float grad2rad(int grad)
  {
    return (2*M_PI*grad)/400;
  }
};


}


#endif
