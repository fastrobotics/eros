#include "BaseWindow.h"
namespace eros_nodes::RemteControl {
bool BaseWindow::update(double /* dt */, double t_ros_time) {
    t_ros_time_ = t_ros_time;
    return true;
}

}  // namespace eros_nodes::RemteControl