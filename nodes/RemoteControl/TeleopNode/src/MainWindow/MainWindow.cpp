#include "MainWindow/MainWindow.h"
namespace eros_nodes::RemoteControl {
constexpr double MainWindow::START_X_PERC;
constexpr double MainWindow::START_Y_PERC;
constexpr double MainWindow::WIDTH_PERC;
constexpr double MainWindow::HEIGHT_PERC;
MainWindow::~MainWindow() {
}
bool MainWindow::update(double dt, double t_ros_time) {
    bool status = BaseWindow::update(dt, t_ros_time);
    if (status == false) {
        return false;
    }
    status = update_window();
    return status;
}
bool MainWindow::update_window() {
    if (get_window() == nullptr) {
        return false;
    }
    // GCOVR_EXCL_START

    return true;
    // GCOVR_EXCL_STOP
}
std::string MainWindow::pretty() {
    std::string str = "-----Main Window-----\n";
    return str;
}
}  // namespace eros_nodes::RemoteControl