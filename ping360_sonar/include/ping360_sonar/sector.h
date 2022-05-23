#ifndef PING360_SONAR_SECTOR
#define PING360_SONAR_SECTOR

#include <map>
#include <math.h>
#include <vector>
#include <stdint.h>


namespace ping360_sonar
{

// helper class to define an image sector
class Sector
{
  static int radius, radius_sq;
  struct Bound
  {
    int x{}, low{}, up{};
    static inline int clamp(double value)
    {
      return value < -radius ? -radius : value > radius ? radius : value;
    }
    explicit Bound(int x, double tan_min, double tan_max)
      : x{x}, low{clamp(x*tan_min)}, up{clamp(x*tan_max)}
    {
      if(up*up + x*x > radius_sq)
        up = clamp((up > 0 ? 1:-1) * sqrt(radius_sq-x*x));

      if(low > up)
        std::swap(low, up);
    }
    explicit Bound(int x, double tan_min, int direction)
      : x{x}, low{clamp(x*tan_min)}, up{clamp(direction * sqrt(radius_sq-x*x))}
    {
      if(low > up)
        std::swap(low, up);
    }
  };

  float dr{};
  std::vector<Bound> bounds;
  std::vector<Bound>::iterator cur;
  std::tuple<int, int, bool> xLimits(float angle_min, float angle_max);

public:

  inline void configure(uint16_t samples, int half_size)
  {
    dr = static_cast<float>(half_size)/samples;
    this->radius = half_size;
    this->radius_sq = half_size*half_size-1;
  }
  void init(float angle, float angle_step);
  bool nextPoint(int &x, int &y, int &index);
};

}


#endif
