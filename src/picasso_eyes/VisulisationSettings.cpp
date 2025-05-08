#include "VisulisationSettings.h"

#include <std_msgs/msg/color_rgba.hpp>
#include <geometry_msgs/msg/vector3.hpp>

namespace Colours {
  std_msgs::msg::ColorRGBA make_colorrgba(float r, float g, float b, float a) {
    std_msgs::msg::ColorRGBA color;
    color.r = r;
    color.g = g;
    color.b = b;
    color.a = a;
    return color;
  }

  const std_msgs::msg::ColorRGBA green = make_colorrgba(0.0f, 1.0f, 0.0f, 1.0f);
  const std_msgs::msg::ColorRGBA red = make_colorrgba(1.0f, 0.0f, 0.0f, 1.0f);
  const std_msgs::msg::ColorRGBA white = make_colorrgba(1.0f, 1.0f, 1.0f, 1.0f);
}

namespace Scales {
  geometry_msgs::msg::Vector3 make_vector3(double x, double y, double z) {
    geometry_msgs::msg::Vector3 scale;
    scale.x = x;
    scale.y = y;
    scale.z = z;
    return scale;
  }

  const geometry_msgs::msg::Vector3 scaleHead = make_vector3(1e-3, 1e-3, 2e-2);
  const geometry_msgs::msg::Vector3 scaleTail = make_vector3(2e-3, 2e-3, 1e-2);
  const geometry_msgs::msg::Vector3 scaleLine = make_vector3(1e-3, 0.0, 0.0);
}