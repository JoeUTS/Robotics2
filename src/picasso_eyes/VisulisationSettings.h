#ifndef VISUILISATIONSETTINGS_H
#define VISUILISATIONSETTINGS_H

#include <std_msgs/msg/color_rgba.hpp>
#include <geometry_msgs/msg/vector3.hpp>

namespace Colours {
  std_msgs::msg::ColorRGBA green;
  green.r = 0;
  green.g = 1;
  green.b = 0;
  green.a = 1;

  std_msgs::msg::ColorRGBA red;
  red.r = 1;
  red.g = 0;
  red.b = 0;
  red.a = 1;

  std_msgs::msg::ColorRGBA white;
  white.r = 1;
  white.g = 1;
  white.b = 1;
  white.a = 1;
}

namespace Scales {
  // TO DO: Finish
  geometry_msgs::msg::Vector3 scaleHead(1e-3, 1e-3, 2e-2);
  geometry_msgs::msg::Vector3 scaleTail(2e-3, 2e-3, 1e-2);
  geometry_msgs::msg::Vector3 scaleLine(1e-3, 0, 0);
}



#endif // VISUILISATIONSETTINGS_H