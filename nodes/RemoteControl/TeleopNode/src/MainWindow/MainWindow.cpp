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
    if (armed_state_.state != eros::ArmDisarm::Type::ARMED) {
        cmd_vel_perc.linear.x = 0.0;
        cmd_vel_perc.linear.y = 0.0;
        cmd_vel_perc.linear.z = 0.0;
        cmd_vel_perc.angular.x = 0.0;
        cmd_vel_perc.angular.y = 0.0;
        cmd_vel_perc.angular.z = 0.0;
    }
    status = update_window();
    cmd_vel_pub.publish(cmd_vel_perc);
    return status;
}
bool MainWindow::new_msg(eros::ArmDisarm::State armed_state) {
    armed_state_ = armed_state;
    return true;
}
bool MainWindow::update_window() {
    if (get_window() == nullptr) {
        return false;
    }
    {
        std::string velocity_linear_str =
            "Vel Cmd: " + std::to_string(cmd_vel_perc.linear.x) + " (%)";
        eros_window::Color velocity_linear_color;
        if (cmd_vel_perc.linear.x > 5.0) {
            velocity_linear_color = eros_window::Color::GREEN_COLOR;
        }
        else if (cmd_vel_perc.linear.x < -5.0) {
            velocity_linear_color = eros_window::Color::RED_COLOR;
        }
        else {
            velocity_linear_color = eros_window::Color::YELLOW_COLOR;
        }
        wattron(get_window(), COLOR_PAIR(velocity_linear_color));
        mvwprintw(get_window(), 2, 1, velocity_linear_str.c_str());
        wclrtoeol(get_window());
        wattroff(get_window(), COLOR_PAIR(velocity_linear_color));
    }
    {
        std::string velocity_angular_str =
            "Rot Cmd: " + std::to_string(cmd_vel_perc.angular.z) + " (%)";
        eros_window::Color velocity_angular_color;
        if (cmd_vel_perc.angular.z > 5.0) {
            velocity_angular_color = eros_window::Color::GREEN_COLOR;
        }
        else if (cmd_vel_perc.angular.z < -5.0) {
            velocity_angular_color = eros_window::Color::RED_COLOR;
        }
        else {
            velocity_angular_color = eros_window::Color::YELLOW_COLOR;
        }
        wattron(get_window(), COLOR_PAIR(velocity_angular_color));
        mvwprintw(get_window(), 3, 1, velocity_angular_str.c_str());
        wclrtoeol(get_window());
        wattroff(get_window(), COLOR_PAIR(velocity_angular_color));
    }

    // GCOVR_EXCL_START
    wclrtobot(get_window());
    box(get_window(), 0, 0);
    wrefresh(get_window());
    return true;
    // GCOVR_EXCL_STOP
}
eros_window::KeyEventContainer MainWindow::new_keyevent(int key) {
    eros_window::KeyEventContainer output;
    if (std::find(supported_keys.begin(), supported_keys.end(), key) != supported_keys.end()) {
        output.message.level =
            eros::Level::Type::ERROR;  // Set default Level to error, so if any supported keys
                                       // are not processed, will actively fail.
    }
    else {
        return output;
    }
    if (armed_state_.state == eros::ArmDisarm::Type::ARMED) {
        geometry_msgs::Twist new_cmd = cmd_vel_perc;
        double MAX_PERCENT = 100.0;
        double step = 10.0;
        bool key_accepted = false;
        if (key == KEY_UP) {
            new_cmd.linear.x += step;
            key_accepted = true;
        }
        else if (key == KEY_DOWN) {
            new_cmd.linear.x -= step;
            key_accepted = true;
        }
        else if (key == KEY_LEFT) {
            new_cmd.angular.z += step;
            key_accepted = true;
        }
        else if (key == KEY_RIGHT) {
            new_cmd.angular.z -= step;
            key_accepted = true;
        }
        if (new_cmd.linear.x > MAX_PERCENT) {
            new_cmd.linear.x = MAX_PERCENT;
        }
        if (new_cmd.linear.x < -1.0 * MAX_PERCENT) {
            new_cmd.linear.x = -1.0 * MAX_PERCENT;
        }
        if (new_cmd.linear.x > MAX_PERCENT) {
            new_cmd.linear.x = MAX_PERCENT;
        }
        if (new_cmd.linear.x < -1.0 * MAX_PERCENT) {
            new_cmd.linear.x = -1.0 * MAX_PERCENT;
        }
        if (new_cmd.angular.z > MAX_PERCENT) {
            new_cmd.angular.z = MAX_PERCENT;
        }
        if (new_cmd.angular.z < -1.0 * MAX_PERCENT) {
            new_cmd.angular.z = -1.0 * MAX_PERCENT;
        }
        cmd_vel_perc = new_cmd;
        if (key_accepted == true) {
            output.message.level = eros::Level::Type::INFO;
        }
    }
    return output;
}
std::string MainWindow::pretty() {
    std::string str = "-----Main Window-----\n";
    return str;
}
}  // namespace eros_nodes::RemoteControl