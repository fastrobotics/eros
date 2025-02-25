#include "SafetyNodeProcess.h"
using namespace eros;
using namespace eros_nodes::Infrastructure;

SafetyNodeProcess::SafetyNodeProcess() {
}
SafetyNodeProcess::~SafetyNodeProcess() {
    cleanup();
}
eros_diagnostic::Diagnostic SafetyNodeProcess::finish_initialization() {
    eros_diagnostic::Diagnostic diag;
    diag = update_diagnostic(eros_diagnostic::DiagnosticType::REMOTE_CONTROL,
                             Level::Type::INFO,
                             eros_diagnostic::Message::NODATA,
                             "No Remote Control Command Yet.");
    return diag;
}
void SafetyNodeProcess::reset() {
}

eros_diagnostic::Diagnostic SafetyNodeProcess::update(double t_dt, double t_ros_time) {
    eros_diagnostic::Diagnostic diag = base_update(t_dt, t_ros_time);
    if (diag.level > Level::Type::WARN) {
        return diag;
    }
    diag = armed_state_manager->update(t_ros_time);
    return diag;
}
std::vector<eros_diagnostic::Diagnostic> SafetyNodeProcess::new_commandmsg(eros::command cmd) {
    std::vector<eros_diagnostic::Diagnostic> diag_list;
    eros_diagnostic::Diagnostic diag = get_root_diagnostic();
    if ((cmd.Command == (uint16_t)Command::Type::ARM) ||
        (cmd.Command == (uint16_t)Command::Type::DISARM)) {
        diag = armed_state_manager->new_command(cmd);
    }
    diag_list.push_back(diag);
    diag = update_diagnostic(diag);
    return diag_list;
}
std::vector<eros_diagnostic::Diagnostic> SafetyNodeProcess::check_programvariables() {
    std::vector<eros_diagnostic::Diagnostic> diag_list;
    return diag_list;
}
bool SafetyNodeProcess::new_message_readytoarm(std::string name, eros::ready_to_arm ready_to_arm) {
    return armed_state_manager->new_ready_to_arm_msg(name, ready_to_arm.ready_to_arm);
}
bool SafetyNodeProcess::new_message_readytoarm(std::string name, bool ready_to_arm) {
    return armed_state_manager->new_ready_to_arm_msg(name, ready_to_arm);
}
bool SafetyNodeProcess::set_ready_to_arm_signals(std::vector<std::string> signals) {
    eros_diagnostic::Diagnostic diag = get_root_diagnostic();
    armed_state_manager = new eros::ArmedStateManager(
        diag.device_name, diag.node_name, diag.system, diag.subsystem, signals);
    return true;
}
std::string SafetyNodeProcess::pretty() {
    std::string str = "Node State: " + Node::NodeStateString(get_nodestate());
    str += armed_state_manager->pretty();
    return str;
}