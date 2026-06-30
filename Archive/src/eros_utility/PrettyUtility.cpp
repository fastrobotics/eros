#include <eros_utility/PrettyUtility.h>
namespace eros::eros_utility {
std::string PrettyUtility::pretty(nav_msgs::Odometry odom) {
    std::string str;
    str = "Odom: T=" + std::to_string(odom.header.stamp.toSec());
    return str;
}
}  // namespace eros::eros_utility