#include "TeleopJoyNodeProcess.h"
using namespace eros;
using namespace eros_nodes::Infrastructure;

TeleopJoyNodeProcess::TeleopJoyNodeProcess() {
}
TeleopJoyNodeProcess::~TeleopJoyNodeProcess() {
    cleanup();
}
eros_diagnostic::Diagnostic TeleopJoyNodeProcess::finish_initialization() {
    eros_diagnostic::Diagnostic diag;
    diag = update_diagnostic(eros_diagnostic::DiagnosticType::REMOTE_CONTROL,
                             Level::Type::INFO,
                             eros_diagnostic::Message::NODATA,
                             "No Remote Control Command Yet.");
    return diag;
}
void TeleopJoyNodeProcess::reset() {
}

eros_diagnostic::Diagnostic TeleopJoyNodeProcess::update(double t_dt, double t_ros_time) {
    eros_diagnostic::Diagnostic diag = base_update(t_dt, t_ros_time);
    if (diag.level > Level::Type::WARN) {
        return diag;
    }
    ready_to_arm.ready_to_arm = true;
    ready_to_arm.diag = eros_diagnostic::DiagnosticUtility::convert(diag);
    return diag;
}
std::vector<eros_diagnostic::Diagnostic> TeleopJoyNodeProcess::new_commandmsg(eros::command cmd) {
    std::vector<eros_diagnostic::Diagnostic> diag_list = base_new_commandmsg(cmd);
    if (diag_list.size() == 0) {
        // No currently supported commands.
    }
    else {
        for (auto diag : diag_list) {
            if (diag.level >= Level::Type::INFO) {
                diagnostic_manager.update_diagnostic(diag);
            }
        }
    }
    return diag_list;
}
std::vector<eros::eros_diagnostic::Diagnostic> TeleopJoyNodeProcess::new_joymsg(
    sensor_msgs::Joy msg) {
    std::vector<eros_diagnostic::Diagnostic> diag_list;
    eros_diagnostic::Diagnostic diag = get_root_diagnostic();
    geometry_msgs::Twist twist;
    twist.linear.x = 100.0 * msg.axes[1];
    twist.angular.z = 100.0 * msg.axes[0];
    cmd_vel_perc = twist;
    if (msg.buttons[0] == 1) {
        if (current_armed_state.state == eros::ArmDisarm::Type::DISARMED) {
            command.command.stamp = ros::Time::now();
            command.command.Command = (uint16_t)eros::Command::Type::ARM;
            command.new_command = true;
        }
        else if (current_armed_state.state == eros::ArmDisarm::Type::ARMED) {
            command.command.stamp = ros::Time::now();
            command.command.Command = (uint16_t)eros::Command::Type::DISARM;
            command.new_command = true;
        }
        else {
            diag = update_diagnostic(eros_diagnostic::DiagnosticType::REMOTE_CONTROL,
                                     Level::Type::WARN,
                                     eros_diagnostic::Message::DIAGNOSTIC_FAILED,
                                     "Arm/Disarm Command Not Allowed.");
            logger->log_diagnostic(diag);
        }
    }
    else if (msg.buttons[1] == 1) {
        command.new_command = true;
        command.command.stamp = ros::Time::now();
        command.command.Command = (uint16_t)eros::Command::Type::GENERATE_SNAPSHOT;
        command.command.Option1 = (uint16_t)eros::Command::GenerateSnapshot_Option1::RUN_MASTER;
    }

    diag_list.push_back(diag);
    diag = update_diagnostic(diag);
    return diag_list;
}
std::vector<eros_diagnostic::Diagnostic> TeleopJoyNodeProcess::check_programvariables() {
    std::vector<eros_diagnostic::Diagnostic> diag_list;
    return diag_list;
}
std::string TeleopJoyNodeProcess::pretty() {
    std::string str = "Node State: " + Node::NodeStateString(get_nodestate());
    return str;
}