#include <eros_window/MessageWindow/MessageWindow.h>
namespace eros_window {
constexpr double MessageWindow::START_X_PERC;
constexpr double MessageWindow::START_Y_PERC;
constexpr double MessageWindow::WIDTH_PERC;
constexpr double MessageWindow::HEIGHT_PERC;
MessageWindow::~MessageWindow() {
}

bool MessageWindow::update(double dt, double t_ros_time) {
    bool status = BaseWindow::update(dt, t_ros_time);
    if (status == false) {
        return false;
    }
    timer_showing_message_in_window += dt;

    status = update_window();
    return status;
}
bool MessageWindow::update_window() {
    if (get_window() == nullptr) {
        return false;
    }
    // GCOVR_EXCL_START
    if (timer_showing_message_in_window > TIME_TO_SHOW_MESSAGES) {
        message_text = "";
        message_text_color = eros_window::Color::NO_COLOR;
    }
    wattron(get_window(), COLOR_PAIR(message_text_color));
    mvwprintw(get_window(), 1, 1, message_text.c_str());
    wclrtoeol(get_window());
    wattroff(get_window(), COLOR_PAIR(message_text_color));
    box(get_window(), 0, 0);
    wrefresh(get_window());
    return true;
    // GCOVR_EXCL_STOP
}
bool MessageWindow::new_msg(eros::command_state msg) {
    if ((msg.CurrentCommand.Command == (uint16_t)eros::Command::Type::GENERATE_SNAPSHOT)) {
        if (msg.CurrentCommand.Option1 ==
            (uint16_t)eros::Command::GenerateSnapshot_Option1::RUN_MASTER) {
            if (msg.State == (uint8_t)eros::SnapshotState::RUNNING) {
                char tempstr[512];
                sprintf(tempstr, "System Snap Progress: %4.2f %%", msg.PercentComplete);
                set_message_text(std::string(tempstr), eros::Level::Type::NOTICE);
            }
            else if (msg.State == (uint8_t)eros::SnapshotState::INCOMPLETE) {
                set_message_text("System Snapshot Incomplete", eros::Level::Type::WARN);
            }
            else if (msg.State == (uint8_t)eros::SnapshotState::COMPLETE) {
                set_message_text("System Snapshot Completed.", eros::Level::Type::NOTICE);
            }
        }
    }
    else {
        set_message_text(msg.diag.Description, (eros::Level::Type)msg.diag.Level);
    }
    return true;
}
bool MessageWindow::new_MessageTextList(std::vector<eros_window::MessageText> messages) {
    if (messages.size() == 0) {
        // Don't do anything
        return true;
    }
    else if (messages.size() == 1) {
        set_message_text(messages.at(0).text, messages.at(0).level);
        return true;
    }
    else {
        logger->log_error("Messages of size: " + std::to_string(messages.size()) +
                          " Not Supported Yet!");
        return false;
    }
}
void MessageWindow::set_message_text(std::string text, eros::Level::Type level) {
    if (text == "") {
        return;
    }
    eros_window::Color color;
    switch (level) {
        case eros::Level::Type::DEBUG: color = eros_window::Color::NO_COLOR; break;
        case eros::Level::Type::INFO: color = eros_window::Color::NO_COLOR; break;
        case eros::Level::Type::NOTICE: color = eros_window::Color::GREEN_COLOR; break;
        case eros::Level::Type::WARN: color = eros_window::Color::YELLOW_COLOR; break;
        case eros::Level::Type::ERROR: color = eros_window::Color::RED_COLOR; break;
        case eros::Level::Type::FATAL: color = eros_window::Color::RED_COLOR; break;
        default: color = eros_window::Color::RED_COLOR; break;
    }
    set_message_text(text, color);
}
void MessageWindow::set_message_text(std::string text, eros_window::Color color) {
    message_text = text;
    message_text_color = color;
    timer_showing_message_in_window = 0.0;
}
}  // namespace eros_window