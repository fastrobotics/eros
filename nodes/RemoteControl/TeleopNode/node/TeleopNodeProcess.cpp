#include "TeleopNodeProcess.h"

#include "MainWindow/MainWindow.h"
using namespace eros;
using namespace eros_nodes::RemoteControl;

TeleopNodeProcess::~TeleopNodeProcess() {
    for (auto window : windows) { delete window; }
    cleanup();
}
eros_diagnostic::Diagnostic TeleopNodeProcess::finish_initialization() {
    eros_diagnostic::Diagnostic diag;
    return diag;
}
void TeleopNodeProcess::reset() {
}
eros_diagnostic::Diagnostic TeleopNodeProcess::update(double t_dt, double t_ros_time) {
    eros_diagnostic::Diagnostic diag = base_update(t_dt, t_ros_time);
    int key_pressed = getch();
    if ((key_pressed == KEY_q) || (key_pressed == KEY_Q)) {
        kill_me = true;
    }
    if (key_pressed != -1) {
        logger->log_warn(std::to_string(key_pressed));
    }
    ready_to_arm.ready_to_arm = true;
    ready_to_arm.diag = eros_diagnostic::DiagnosticUtility::convert(diag);
    return diag;
}
std::vector<eros_diagnostic::Diagnostic> TeleopNodeProcess::new_commandmsg(eros::command msg) {
    (void)msg;  // Not used yet.
    std::vector<eros_diagnostic::Diagnostic> diag_list;
    return diag_list;
}
std::vector<eros_diagnostic::Diagnostic> TeleopNodeProcess::check_programvariables() {
    std::vector<eros_diagnostic::Diagnostic> diag_list;
    return diag_list;
}
std::string TeleopNodeProcess::pretty() {
    std::string str = "Node State: " + Node::NodeStateString(get_nodestate());
    return str;
}
bool TeleopNodeProcess::initialize_windows() {
    timeout(0);
    keypad(stdscr, TRUE);
    {
        IWindow* window = new MainWindow(
            nodeHandle, robot_namespace, logger, mainwindow_height, mainwindow_width);
        windows.push_back(window);
    }
    return true;
}