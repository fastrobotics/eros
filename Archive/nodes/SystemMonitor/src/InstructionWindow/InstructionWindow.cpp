#include "InstructionWindow/InstructionWindow.h"

#include <eros/command.h>
namespace eros_nodes::SystemMonitor {
constexpr double InstructionWindow::START_X_PERC;
constexpr double InstructionWindow::START_Y_PERC;
constexpr double InstructionWindow::WIDTH_PERC;
constexpr double InstructionWindow::HEIGHT_PERC;
InstructionWindow::~InstructionWindow() {
}

bool InstructionWindow::update(double dt, double t_ros_time) {
    bool status = BaseWindow::update(dt, t_ros_time);
    if (status == false) {
        return false;
    }
    status = update_window();
    return status;
}
bool InstructionWindow::update_window() {
    if (get_window() == nullptr) {
        return false;
    }
    // GCOVR_EXCL_START
    std::vector<std::string> instruction_string;
    // Instructions that are always supported
    instruction_string.push_back("Esc: Reset Screen.  SPACE: Arm/Disarm.");
    instruction_string.push_back("S: Start System Snapshot. (C: Clear Snapshots)");
    if (diagnostic_mode == DiagnosticMode::SYSTEM) {
        instruction_string.push_back("D: View NODE Diagnostics");
    }
    else if (diagnostic_mode == DiagnosticMode::NODE) {
        instruction_string.push_back("D: View SYSTEM Diagnostics");
    }
    if (instruction_mode == InstructionMode::NODE) {
        instruction_string.push_back("F: Get Node Firmware.");
        instruction_string.push_back("L: Change Log Level.");
        instruction_string.push_back("N: Change Node State (1-9).");
    }

    else {
        logger->log_warn("Mode: " + std::to_string((uint8_t)instruction_mode) + " Not Supported!");
        return false;
    }
    for (std::size_t i = 0; i < instruction_string.size(); ++i) {
        mvwprintw(get_window(), i + 3, 1, instruction_string.at(i).c_str());
        wclrtoeol(get_window());
    }
    wclrtobot(get_window());
    box(get_window(), 0, 0);
    wrefresh(get_window());
    return true;
    // GCOVR_EXCL_STOP
}
eros_window::KeyEventContainer InstructionWindow::new_keyevent(int key) {
    eros_window::KeyEventContainer output;
    if (std::find(supported_keys.begin(), supported_keys.end(), key) != supported_keys.end()) {
        output.message.level =
            eros::Level::Type::ERROR;  // Set default Level to error, so if any supported keys
                                       // are not processed, will actively fail.
    }
    else {
        return output;
    }
    // Keys that are always supported.
    if ((key == eros_window::KEY_esc)) {
        output.command.type = eros_window::WindowCommandType::VIEW_DIAGNOSTICS_SYSTEM;
        std::string str = "Requesting Diagnostics for System";
        eros_window::MessageText message(str, eros::Level::Type::INFO);
        logger->log_debug(str);
        output.message = message;
        return output;
    }
    else if (key == eros_window::KEY_space) {
        if (current_armed_state.state == eros::ArmDisarm::Type::DISARMED) {
            eros_window::MessageText message("Arming Robot...", eros::Level::Type::INFO);
            eros::command command;
            command.stamp = ros::Time::now();
            command.Command = (uint16_t)eros::Command::Type::ARM;
            command_pub.publish(command);
            output.message = message;
            return output;
        }
        else if (current_armed_state.state == eros::ArmDisarm::Type::ARMED) {
            eros_window::MessageText message("Disarming Robot...", eros::Level::Type::INFO);
            eros::command command;
            command.stamp = ros::Time::now();
            command.Command = (uint16_t)eros::Command::Type::DISARM;
            command_pub.publish(command);
            output.message = message;
            return output;
        }
        else {
            eros_window::MessageText message("Arm/Disarm Command Not Allowed.",
                                             eros::Level::Type::WARN);
            output.message = message;
            return output;
        }
    }
    else if ((key == eros_window::KEY_s) || (key == eros_window::KEY_S)) {
        eros_window::MessageText message("Requesting System Snapshot...", eros::Level::Type::INFO);
        eros::command command;
        command.stamp = ros::Time::now();
        command.Command = (uint16_t)eros::Command::Type::GENERATE_SNAPSHOT;
        command.Option1 = (uint16_t)eros::Command::GenerateSnapshot_Option1::RUN_MASTER;
        command_pub.publish(command);
        output.message = message;
        return output;
    }
    else if ((key == eros_window::KEY_c) || (key == eros_window::KEY_C)) {
        eros_window::MessageText message("Clearing All Snapshots...", eros::Level::Type::WARN);
        eros::command command;
        command.stamp = ros::Time::now();
        command.Command = (uint16_t)eros::Command::Type::GENERATE_SNAPSHOT;
        command.Option1 = (uint16_t)eros::Command::GenerateSnapshot_Option1::CLEAR_SNAPSHOTS;
        command_pub.publish(command);
        output.message = message;
        return output;
    }
    else {
        return output;
    }
    /*
    // Specific Key/Mode support
    if (instruction_mode == InstructionMode::NODE) {}
    else {
        logger->log_warn("Mode: " + std::to_string((uint8_t)instruction_mode) + " Not Supported!");
        return output;
    }
    return output;
    */
}
}  // namespace eros_nodes::SystemMonitor