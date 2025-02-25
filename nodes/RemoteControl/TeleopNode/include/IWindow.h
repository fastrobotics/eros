#pragma once

#include "WindowDefinitions.h"
namespace eros_nodes::RemoteControl {
/**
 * @brief Interface class for generic Windows
 *
 */
class IWindow
{
   public:
    virtual ~IWindow() {
    }

    virtual bool update(double dt, double t_ros_time) = 0;
    virtual KeyEventContainer new_keyevent(int key) = 0;
};
}  // namespace eros_nodes::RemoteControl