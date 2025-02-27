#include "TeleopNodeProcess.h"

using namespace eros;
namespace eros_nodes::RemoteControl {
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
    if ((key_pressed == eros_window::KEY_q) || (key_pressed == eros_window::KEY_Q)) {
        kill_me = true;
    }
    else if ((key_pressed == eros_window::KEY_tab)) {
        int16_t current_tab_index = tab_index;
        int16_t new_tab_index = current_tab_index + 1;
        if (new_tab_index >= highest_tab_index) {
            new_tab_index = 0;
        }
        tab_index = new_tab_index;
    }
    std::vector<eros_window::MessageText> messages;
    std::vector<eros_window::WindowCommand> window_commands;
    std::string previous_selected_window;
    for (auto window : windows) {
        if (window->has_focus()) {
            previous_selected_window = window->get_name();
        }
    }
    std::string new_selected_window;
    for (auto window : windows) {
        if (window->is_selectable()) {
            if (window->get_tab_order() == tab_index) {
                window->set_focused(true);
                new_selected_window = window->get_name();
            }
            else {
                window->set_focused(false);
            }
        }
        auto output = window->new_keyevent(key_pressed);
        if (output.message.text != "") {
            messages.push_back(output.message);
        }
        if (output.command.type != eros_window::WindowCommandType::UNKNOWN) {
            window_commands.push_back(output.command);
        }
    }
    /*
    if (previous_selected_window != new_selected_window) {
        for (auto window : windows) {
            auto* p = dynamic_cast<InstructionWindow*>(
                window);  // Figure out which window is actually a Message Window
            if (p) {
                if (new_selected_window == "node_window") {
                    p->set_InstructionMode(InstructionWindow::InstructionMode::NODE);
                }
            }
        }
    }
    */
    // Update Message Window only if there's new messages
    if (messages.size() > 0) {
        for (auto window : windows) {
            auto* p = dynamic_cast<eros_window::MessageWindow*>(
                window);  // Figure out which window is actually a Message Window
            if (p) {
                bool status = p->new_MessageTextList(messages);
                if (status == false) {
                    diag = diagnostic_manager.update_diagnostic(
                        eros_diagnostic::DiagnosticType::SOFTWARE,
                        Level::Type::ERROR,
                        eros_diagnostic::Message::DROPPING_PACKETS,
                        "Unable to update Window: " + window->get_name() +
                            " With new Message Text List.");
                }
            }
        }
    }
    // Update all Windows
    for (auto window : windows) {
        window->new_command(window_commands);
        window->update(t_dt, t_ros_time);
    }
    flushinp();
    ready_to_arm.ready_to_arm = true;
    ready_to_arm.diag = eros_diagnostic::DiagnosticUtility::convert(diag);
    return diag;
}
std::vector<eros_diagnostic::Diagnostic> TeleopNodeProcess::new_commandmsg(eros::command msg) {
    (void)msg;  // Not used yet.
    std::vector<eros_diagnostic::Diagnostic> diag_list;
    return diag_list;
}
eros::eros_diagnostic::Diagnostic TeleopNodeProcess::new_commandstate(
    const eros::command_state::ConstPtr& t_msg) {
    eros::command_state msg = eros_utility::ConvertUtility::convert_fromptr(t_msg);
    eros::eros_diagnostic::Diagnostic diag = get_root_diagnostic();

    for (auto window : windows) {
        bool status = window->new_msg(msg);
        if (status == false) {
            diag = diagnostic_manager.update_diagnostic(
                eros_diagnostic::DiagnosticType::SOFTWARE,
                Level::Type::ERROR,
                eros_diagnostic::Message::DROPPING_PACKETS,
                "Unable to update Window: " + window->get_name() + " With new Command State Msg.");
        }
    }
    return diag;
}
void TeleopNodeProcess::update_armedstate(eros::ArmDisarm::State armed_state) {
    eros::eros_diagnostic::Diagnostic diag = get_root_diagnostic();
    for (auto window : windows) {
        bool status = window->new_msg(armed_state);
        if (status == false) {
            diag = diagnostic_manager.update_diagnostic(
                eros_diagnostic::DiagnosticType::SOFTWARE,
                Level::Type::ERROR,
                eros_diagnostic::Message::DROPPING_PACKETS,
                "Unable to update Window: " + window->get_name() + " With new Armed State.");
        }
    }
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
        eros_window::IWindow* window = new MainWindow(
            nodeHandle, robot_namespace, logger, -1, mainwindow_height, mainwindow_width);
        windows.push_back(window);
    }
    {
        eros_window::IWindow* window = new eros_window::HeaderWindow(
            nodeHandle, robot_namespace, logger, -1, mainwindow_height, mainwindow_width);
        windows.push_back(window);
    }
    {
        eros_window::IWindow* window = new eros_window::MessageWindow(
            nodeHandle, robot_namespace, logger, -1, mainwindow_height, mainwindow_width);
        windows.push_back(window);
    }
    {
        eros_window::IWindow* window = new InstructionWindow(nodeHandle,
                                                             robot_namespace,
                                                             logger,
                                                             -1,
                                                             mainwindow_height,
                                                             mainwindow_width,
                                                             command_pub);
        windows.push_back(window);
    }
    {
        eros_window::IWindow* window = new eros_window::DiagnosticsWindow(
            nodeHandle, robot_namespace, logger, -1, mainwindow_height, mainwindow_width);
        windows.push_back(window);
    }
    return true;
}
}  // namespace eros_nodes::RemoteControl