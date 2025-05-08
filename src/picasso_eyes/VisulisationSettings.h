#ifndef VISUILISATIONSETTINGS_H
#define VISUILISATIONSETTINGS_H

#include <std_msgs/msg/color_rgba.hpp>
#include <geometry_msgs/msg/vector3.hpp>

namespace Colours {
  extern const std_msgs::msg::ColorRGBA green;
  extern const std_msgs::msg::ColorRGBA red;
  extern const std_msgs::msg::ColorRGBA white;
}

namespace Scales {
  extern const geometry_msgs::msg::Vector3 scaleHead;
  extern const geometry_msgs::msg::Vector3 scaleTail;
  extern const geometry_msgs::msg::Vector3 scaleLine;
}

#endif // VISUILISATIONSETTINGS_H