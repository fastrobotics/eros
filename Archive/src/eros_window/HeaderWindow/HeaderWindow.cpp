#include <eros_window/HeaderWindow/HeaderWindow.h>
namespace eros_window {
constexpr double HeaderWindow::START_X_PERC;
constexpr double HeaderWindow::START_Y_PERC;
constexpr double HeaderWindow::WIDTH_PERC;
constexpr double HeaderWindow::HEIGHT_PERC;
HeaderWindow::~HeaderWindow() {
}
bool HeaderWindow::new_msg(eros::ArmDisarm::State armed_state) {
    armed_state_ = armed_state;
    return true;
}
bool HeaderWindow::update(double dt, double t_ros_time) {
    bool status = BaseWindow::update(dt, t_ros_time);
    if (status == false) {
        return false;
    }
    status = update_window();
    return status;
}
bool HeaderWindow::update_window() {
    if (get_window() == nullptr) {
        return false;
    }
    // GCOVR_EXCL_START
    {  // Armed State

        eros_window::Color color;
        std::string str = "Armed State: " + eros::ArmDisarm::ArmDisarmString(armed_state_.state);
        str.insert(str.end(), 40 - str.size(), ' ');
        switch (armed_state_.state) {
            case eros::ArmDisarm::Type::ARMED:
                color = eros_window::Color::BLUE_COLOR;
                break;  // Should be BLUE for RC Mode, GREEN for Manual, PURPLE for Auto
            case eros::ArmDisarm::Type::DISARMED_CANNOTARM:
                color = eros_window::Color::RED_COLOR;
                break;
            case eros::ArmDisarm::Type::DISARMED: color = eros_window::Color::GREEN_COLOR; break;
            case eros::ArmDisarm::Type::DISARMING: color = eros_window::Color::GREEN_COLOR; break;
            case eros::ArmDisarm::Type::ARMING: color = eros_window::Color::GREEN_COLOR; break;
            default: color = eros_window::Color::RED_COLOR; break;
        }
        wattron(get_window(), COLOR_PAIR(color));
        mvwprintw(get_window(), 2, 1, str.c_str());
        wclrtoeol(get_window());
        wattroff(get_window(), COLOR_PAIR(color));
    }

    box(get_window(), 0, 0);
    wrefresh(get_window());
    return true;
    // GCOVR_EXCL_STOP
}
}  // namespace eros_window