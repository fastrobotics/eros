#pragma once
#include <nav_msgs/Odometry.h>
namespace eros::eros_utility {
class PrettyUtility
{
   public:
    static std::string pretty(nav_msgs::Odometry odom);
};
}  // namespace eros::eros_utility