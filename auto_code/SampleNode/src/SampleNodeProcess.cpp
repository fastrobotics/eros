#include "SampleNodeProcess.h"

using namespace eros;
namespace SamplePackage {
SampleNodeProcess::SampleNodeProcess() {
}
SampleNodeProcess::~SampleNodeProcess() {
}
eros_diagnostic::Diagnostic SampleNodeProcess::finish_initialization() {
    eros_diagnostic::Diagnostic diag = get_root_diagnostic();
    return diag;
}
void SampleNodeProcess::reset() {
}
eros_diagnostic::Diagnostic SampleNodeProcess::update(double t_dt, double t_ros_time) {
    eros_diagnostic::Diagnostic diag = base_update(t_dt, t_ros_time);
    return diag;
}
std::vector<eros_diagnostic::Diagnostic> SampleNodeProcess::new_commandmsg(eros::command msg) {
    (void)msg;
    std::vector<eros_diagnostic::Diagnostic> diag_list;
    logger->log_warn("No Command Messages Supported at this time.");
    return diag_list;
}
std::vector<eros_diagnostic::Diagnostic> SampleNodeProcess::check_programvariables() {
    std::vector<eros_diagnostic::Diagnostic> diag_list;
    logger->log_warn("No Program Variables Checked.");
    return diag_list;
}
std::string SampleNodeProcess::pretty() {
    std::string str = "SampleNodeProcess";
    return str;
}
}  // namespace SamplePackage
