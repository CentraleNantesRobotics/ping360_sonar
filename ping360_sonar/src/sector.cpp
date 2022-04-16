#include <ping360_sonar/sector.h>
#include <iostream>

using namespace ping360_sonar;

int Sector::radius;
int Sector::radius_sq;

void Sector::init(float angle, float angle_step)
{
  const auto angle_min{angle-angle_step/2};
  const auto angle_max{angle+angle_step/2};

  // get x-bounds for this angular range
  const auto &[xmin, xmax,same_side] = xLimits(angle_min, angle_max); {}
  bounds.clear();
  bounds.reserve(xmax-xmin+1);

  // get corresponding y-bounds
  auto tm{tan(angle_min)};
  auto tM{tan(angle_max)};

  if(same_side)
  {
    if(std::abs(tm) > std::abs(tM))
      std::swap(tm,tM);
    for(auto x = xmin; x <= xmax; ++x)
      bounds.emplace_back(x, tm, tM);
  }
  else
  {
    // forward around pi/2, backward around -pi/2
    const auto direction = fabs(angle-M_PI/2) < fabs(angle+M_PI/2) ? 1 : -1;
    if(direction == -1)
      std::swap(tm,tM);
    for(auto x = xmin; x < 0; ++x)
      bounds.emplace_back(x, tM, direction);
    for(auto x = 0; x <= xmax; ++x)
      bounds.emplace_back(x, tm, direction);
  }

  cur = bounds.end();
}

std::tuple<int, int,bool> Sector::xLimits(float angle_min, float angle_max)
{
  auto cm{cos(angle_min)}, cM{cos(angle_max)};
  const auto same_side{cm*cM >= 0};
  if(cM < cm)
    std::swap(cm,cM);
  if(cm * cM > 0)
  {
    if(cM < 0)
      cM = 0;
    else
      cm = 0;
  }
  return {Bound::clamp(radius*cm),
        Bound::clamp(radius*cM),
        same_side};
}

bool Sector::nextPoint(int &x, int &y, int &index)
{
  if(cur == bounds.end())
  {
    cur = bounds.begin();
    x = cur->x;
    y = cur->low;
  }
  else if(y < cur->up)
  {
    y++;
  }
  else
  {
    cur++;
    if(cur == bounds.end())
      return false;
    x = cur->x;
    y = cur->low;
  }
  // find corresponding index
  const auto dist{sqrt(x*x+y*y)};
  index = std::round(dist/dr);
  return true;
}
