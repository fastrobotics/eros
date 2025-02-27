#include "DataLoggerProcess.h"
using namespace eros;
using namespace eros_nodes::Infrastructure;
DataLoggerProcess::DataLoggerProcess()
    : log_directory(""),
      log_directory_available(false),
      logfile_duration(-1.0),
      logging_enabled(false),
      snapshot_mode(true) {
}

DataLoggerProcess::~DataLoggerProcess() {
    cleanup();
}
eros_diagnostic::Diagnostic DataLoggerProcess::finish_initialization() {
    eros_diagnostic::Diagnostic diag;
    return diag;
}
void DataLoggerProcess::reset() {
}
eros_diagnostic::Diagnostic DataLoggerProcess::update(double t_dt, double t_ros_time) {
    eros_diagnostic::Diagnostic diag = base_update(t_dt, t_ros_time);
    ready_to_arm.ready_to_arm = true;
    ready_to_arm.diag = eros_diagnostic::DiagnosticUtility::convert(diag);
    return diag;
}
std::vector<eros_diagnostic::Diagnostic> DataLoggerProcess::new_commandmsg(eros::command msg) {
    (void)msg;  // Not used yet.
    std::vector<eros_diagnostic::Diagnostic> diag_list;
    return diag_list;
}
std::vector<eros_diagnostic::Diagnostic> DataLoggerProcess::check_programvariables() {
    std::vector<eros_diagnostic::Diagnostic> diag_list;
    return diag_list;
}
std::string DataLoggerProcess::pretty() {
    std::string str = "Node State: " + Node::NodeStateString(get_nodestate());
    return str;
}