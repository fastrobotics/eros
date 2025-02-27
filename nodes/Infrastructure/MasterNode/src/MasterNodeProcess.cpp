#include "MasterNodeProcess.h"
using namespace eros;
using namespace eros_nodes::Infrastructure;
MasterNodeProcess::MasterNodeProcess() {
}
MasterNodeProcess::~MasterNodeProcess() {
}
eros_diagnostic::Diagnostic MasterNodeProcess::finish_initialization() {
    eros_diagnostic::Diagnostic diag = get_root_diagnostic();
    return diag;
}
void MasterNodeProcess::reset() {
}
eros_diagnostic::Diagnostic MasterNodeProcess::update(double t_dt, double t_ros_time) {
    eros_diagnostic::Diagnostic diag = base_update(t_dt, t_ros_time);
    ready_to_arm.ready_to_arm = true;
    ready_to_arm.diag = eros_diagnostic::DiagnosticUtility::convert(diag);
    return diag;
}
std::vector<eros_diagnostic::Diagnostic> MasterNodeProcess::new_commandmsg(eros::command msg) {
    (void)msg;  // Currently Unused
    std::vector<eros_diagnostic::Diagnostic> diag_list;
    logger->log_warn("No Command Messages Supported at this time.");
    return diag_list;
}
std::vector<eros_diagnostic::Diagnostic> MasterNodeProcess::check_programvariables() {
    std::vector<eros_diagnostic::Diagnostic> diag_list;
    logger->log_warn("No Program Variables Checked.");
    return diag_list;
}
std::string MasterNodeProcess::pretty() {
    std::string str = "Node State: " + Node::NodeStateString(get_nodestate());
    return str;
}
