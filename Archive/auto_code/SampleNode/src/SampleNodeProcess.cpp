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
    std::vector<eros_diagnostic::Diagnostic> diag_list = base_new_commandmsg(msg);
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
