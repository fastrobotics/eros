#include "SystemMonitorProcess.h"

using namespace eros;
namespace eros_nodes::SystemMonitor {
SystemMonitorProcess::~SystemMonitorProcess() {
    for (auto window : windows) { delete window; }
    cleanup();
}
eros_diagnostic::Diagnostic SystemMonitorProcess::finish_initialization() {
    eros_diagnostic::Diagnostic diag = diagnostic_manager.get_root_diagnostic();
    diag = diagnostic_manager.update_diagnostic(eros_diagnostic::DiagnosticType::SOFTWARE,
                                                Level::Type::INFO,
                                                eros_diagnostic::Message::NOERROR,
                                                "Finished Initialization.");
    return diag;
}
void SystemMonitorProcess::reset() {
}
eros::eros_diagnostic::Diagnostic SystemMonitorProcess::update_monitorlist(
    std::vector<std::string> heartbeat_list,
    std::vector<std::string> resourceused_list,
    std::vector<std::string> resourceavailable_list,
    std::vector<std::string> loadfactor_list,
    std::vector<std::string>& new_heartbeat_topics_to_subscribe,
    std::vector<std::string>& new_resourceused_topics_to_subscribe,
    std::vector<std::string>& new_resourceavailable_topics_to_subscribe,
    std::vector<std::string>& new_loadfactor_topics_to_subscribe) {
    eros_diagnostic::Diagnostic diag = diagnostic_manager.get_root_diagnostic();
    // Check for new Heartbeat messages
    for (auto heartbeat : heartbeat_list) {
        bool found_it = false;
        for (auto monitored_heartbeat : monitored_heartbeat_topics) {
            if (monitored_heartbeat == heartbeat) {
                found_it = true;
                break;
            }
        }
        if (found_it == false) {
            monitored_heartbeat_topics.push_back(heartbeat);
            new_heartbeat_topics_to_subscribe.push_back(heartbeat);
        }
    }
    // Check for new Resource Used messages
    for (auto resourceused : resourceused_list) {
        bool found_it = false;
        for (auto monitored_resourceused : monitored_resourceused_topics) {
            if (monitored_resourceused == resourceused) {
                found_it = true;
                break;
            }
        }
        if (found_it == false) {
            monitored_resourceused_topics.push_back(resourceused);
            new_resourceused_topics_to_subscribe.push_back(resourceused);
        }
    }
    // Check for new Resource Available messages
    for (auto resourceavailable : resourceavailable_list) {
        bool found_it = false;
        for (auto monitored_resourceavailable : monitored_resourceavailable_topics) {
            if (monitored_resourceavailable == resourceavailable) {
                found_it = true;
                break;
            }
        }
        if (found_it == false) {
            monitored_resourceavailable_topics.push_back(resourceavailable);
            new_resourceavailable_topics_to_subscribe.push_back(resourceavailable);
        }
    }
    // Check for new LoadFactor messages
    for (auto loadfactor : loadfactor_list) {
        bool found_it = false;
        for (auto monitored_loadfactor : monitored_loadfactor_topics) {
            if (monitored_loadfactor == loadfactor) {
                found_it = true;
                break;
            }
        }
        if (found_it == false) {
            monitored_loadfactor_topics.push_back(loadfactor);
            new_loadfactor_topics_to_subscribe.push_back(loadfactor);
        }
    }
    return diag;
}
eros_diagnostic::Diagnostic SystemMonitorProcess::update(double t_dt, double t_ros_time) {
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

    return diag;
}
std::vector<eros_diagnostic::Diagnostic> SystemMonitorProcess::new_commandmsg(eros::command msg) {
    (void)msg;  // Not currently used.
    std::vector<eros_diagnostic::Diagnostic> diag_list;
    eros_diagnostic::Diagnostic diag = diagnostic_manager.get_root_diagnostic();
    diag = diagnostic_manager.update_diagnostic(eros_diagnostic::DiagnosticType::SOFTWARE,
                                                Level::Type::INFO,
                                                eros_diagnostic::Message::NOERROR,
                                                "Processed Command.");
    diag_list.push_back(diag);
    return diag_list;
}
std::vector<eros_diagnostic::Diagnostic> SystemMonitorProcess::check_programvariables() {
    std::vector<eros_diagnostic::Diagnostic> diag_list;
    eros_diagnostic::Diagnostic diag = diagnostic_manager.get_root_diagnostic();
    diag = diagnostic_manager.update_diagnostic(eros_diagnostic::DiagnosticType::SOFTWARE,
                                                Level::Type::INFO,
                                                eros_diagnostic::Message::NOERROR,
                                                "Program Variable Check Passed.");
    diag_list.push_back(diag);
    return diag_list;
}
// GCOVR_EXCL_START
bool SystemMonitorProcess::initialize_windows() {
    timeout(0);
    keypad(stdscr, TRUE);
    {
        eros_window::IWindow* window = new NodeWindow(
            nodeHandle, robot_namespace, logger, 0, mainwindow_height, mainwindow_width);
        window->set_focused(true);  // Window defaults to focused
        highest_tab_index++;
        windows.push_back(window);
    }
    {
        eros_window::IWindow* window = new eros_window::DiagnosticsWindow(
            nodeHandle, robot_namespace, logger, 1, mainwindow_height, mainwindow_width);
        highest_tab_index++;
        windows.push_back(window);
    }
    {
        eros_window::IWindow* window = new DeviceWindow(
            nodeHandle, robot_namespace, logger, 2, mainwindow_height, mainwindow_width);
        highest_tab_index++;
        windows.push_back(window);
    }
    {
        eros_window::IWindow* window = new eros_window::HeaderWindow(
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
        eros_window::IWindow* window = new eros_window::MessageWindow(
            nodeHandle, robot_namespace, logger, -1, mainwindow_height, mainwindow_width);
        windows.push_back(window);
    }
    {
        eros_window::IWindow* window = new eros_window::StatusWindow(
            nodeHandle, robot_namespace, logger, -1, mainwindow_height, mainwindow_width);
        windows.push_back(window);
    }

    return true;
}
// GCOVR_EXCL_STOP
eros::eros_diagnostic::Diagnostic SystemMonitorProcess::new_heartbeatmessage(
    const eros::heartbeat::ConstPtr& t_msg) {
    eros::heartbeat msg = eros_utility::ConvertUtility::convert_fromptr(t_msg);
    eros::eros_diagnostic::Diagnostic diag = get_root_diagnostic();
    for (auto window : windows) {
        bool status = window->new_msg(msg);
        if (status == false) {
            diag = diagnostic_manager.update_diagnostic(
                eros_diagnostic::DiagnosticType::SOFTWARE,
                Level::Type::ERROR,
                eros_diagnostic::Message::DROPPING_PACKETS,
                "Unable to update Window: " + window->get_name() + " With new Heartbeat.");
        }
    }
    return diag;
}
eros::eros_diagnostic::Diagnostic SystemMonitorProcess::new_commandstate(
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
void SystemMonitorProcess::update_armedstate(eros::ArmDisarm::State armed_state) {
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
eros::eros_diagnostic::Diagnostic SystemMonitorProcess::new_resourceusedmessage(
    const eros::resource::ConstPtr& t_msg) {
    eros::resource msg = eros_utility::ConvertUtility::convert_fromptr(t_msg);
    eros::eros_diagnostic::Diagnostic diag = get_root_diagnostic();
    for (auto window : windows) {
        if (window->get_name() != "device_window") {
            bool status = window->new_msg(msg);
            if (status == false) {
                diag = diagnostic_manager.update_diagnostic(
                    eros_diagnostic::DiagnosticType::SOFTWARE,
                    Level::Type::ERROR,
                    eros_diagnostic::Message::DROPPING_PACKETS,
                    "Unable to update Window: " + window->get_name() + " With new Resource Used.");
            }
        }
    }
    return diag;
}
eros::eros_diagnostic::Diagnostic SystemMonitorProcess::new_loadfactormessage(
    const eros::loadfactor::ConstPtr& t_msg) {
    eros::loadfactor msg = eros_utility::ConvertUtility::convert_fromptr(t_msg);
    eros::eros_diagnostic::Diagnostic diag = get_root_diagnostic();
    for (auto window : windows) {
        bool status = window->new_msg(msg);
        if (status == false) {
            diag = diagnostic_manager.update_diagnostic(
                eros_diagnostic::DiagnosticType::SOFTWARE,
                Level::Type::ERROR,
                eros_diagnostic::Message::DROPPING_PACKETS,
                "Unable to update Window: " + window->get_name() + " With new Load Factor Data.");
        }
    }
    return diag;
}
eros::eros_diagnostic::Diagnostic SystemMonitorProcess::new_resourceavailablemessage(
    const eros::resource::ConstPtr& t_msg) {
    eros::resource msg = eros_utility::ConvertUtility::convert_fromptr(t_msg);
    eros::eros_diagnostic::Diagnostic diag = get_root_diagnostic();
    for (auto window : windows) {
        if (window->get_name() == "device_window") {
            bool status = window->new_msg(msg);
            if (status == false) {
                diag = diagnostic_manager.update_diagnostic(
                    eros_diagnostic::DiagnosticType::SOFTWARE,
                    Level::Type::ERROR,
                    eros_diagnostic::Message::DROPPING_PACKETS,
                    "Unable to update Window: " + window->get_name() +
                        " With new Resource Available.");
            }
        }
    }
    return diag;
}
std::string SystemMonitorProcess::pretty() {
    std::string str = "Node State: " + Node::NodeStateString(get_nodestate());
    str += "\n----- System Monitor Node Process -----\n";
    str += "Monitored Topics:Heartbeat(" +
           std::to_string((uint16_t)monitored_heartbeat_topics.size()) + ")\n";
    for (auto v : monitored_heartbeat_topics) { str += "\t" + v + "\n"; }
    str += "Monitored Topics:Resource Used(" +
           std::to_string((uint16_t)monitored_resourceused_topics.size()) + ")\n";
    for (auto v : monitored_resourceused_topics) { str += "\t" + v + "\n"; }
    str += "Monitored Topics:Resource Available(" +
           std::to_string((uint16_t)monitored_resourceavailable_topics.size()) + ")\n";
    for (auto v : monitored_resourceavailable_topics) { str += "\t" + v + "\n"; }
    str += "Monitored Topics:Load Factor(" +
           std::to_string((uint16_t)monitored_loadfactor_topics.size()) + ")\n";
    for (auto v : monitored_loadfactor_topics) { str += "\t" + v + "\n"; }
    return str;
}
}  // namespace eros_nodes::SystemMonitor