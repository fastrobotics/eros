#pragma once

#include "CommonWindowDefinitions.h"
namespace eros_window {
/**
 * @brief Interface class for generic Windows
 *
 */
class ICommonWindow
{
   public:
    virtual ~ICommonWindow() {
    }

    virtual bool update(double dt, double t_ros_time) = 0;
    virtual std::vector<int> get_supported_keys() = 0;
};
}  // namespace eros_window