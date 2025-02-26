#include "TeleopNode.h"
using namespace eros;
using namespace eros_nodes::RemoteControl;
bool kill_node = false;
// Coverage Suppress not working for this function.
TeleopNode::~TeleopNode() {
}
TeleopNode::TeleopNode()
    : system_command_action_server(
          *n.get(),
          read_robotnamespace() + "SystemCommandAction",
          boost::bind(&TeleopNode::system_commandAction_Callback, this, _1),
          false) {
    system_command_action_server.start();
}
void TeleopNode::system_commandAction_Callback(const eros::system_commandGoalConstPtr &goal) {
    eros_diagnostic::Diagnostic diag = process->get_root_diagnostic();
    eros::system_commandResult result_;
    system_command_action_server.setAborted(result_);
    diag = process->update_diagnostic(eros_diagnostic::DiagnosticType::COMMUNICATIONS,
                                      Level::Type::WARN,
                                      eros_diagnostic::Message::DROPPING_PACKETS,
                                      "Received unsupported CommandAction: " +
                                          Command::CommandString((Command::Type)goal->Command));
    logger->log_diagnostic(diag);
}
void TeleopNode::command_Callback(const eros::command::ConstPtr &t_msg) {
    eros::command cmd = eros_utility::ConvertUtility::convert_fromptr(t_msg);
    eros_diagnostic::Diagnostic diag = process->get_root_diagnostic();
    diag = process->update_diagnostic(
        eros_diagnostic::DiagnosticType::COMMUNICATIONS,
        Level::Type::WARN,
        eros_diagnostic::Message::DROPPING_PACKETS,
        "Received unsupported Command: " + Command::CommandString((Command::Type)cmd.Command));
    logger->log_diagnostic(diag);
}
bool TeleopNode::changenodestate_service(eros::srv_change_nodestate::Request &req,
                                         eros::srv_change_nodestate::Response &res) {
    Node::State req_state = Node::NodeState(req.RequestedNodeState);
    process->request_statechange(req_state);
    res.NodeState = Node::NodeStateString(process->get_nodestate());
    return true;
}
bool TeleopNode::start() {
    set_no_launch_enabled(true);
    initialize_diagnostic(DIAGNOSTIC_SYSTEM, DIAGNOSTIC_SUBSYSTEM, DIAGNOSTIC_COMPONENT);
    bool status = false;
    process = new TeleopNodeProcess();
    set_basenodename(BASE_NODE_NAME);
    initialize_firmware(
        MAJOR_RELEASE_VERSION, MINOR_RELEASE_VERSION, BUILD_NUMBER, FIRMWARE_DESCRIPTION);
    enable_ready_to_arm_pub(true);
    diagnostic = preinitialize_basenode();
    // No Practical way to Unit Test
    // LCOV_EXCL_START
    if (diagnostic.level > Level::Type::WARN) {
        return false;
    }
    // LCOV_EXCL_STOP
    diagnostic = read_launchparameters();
    // No Practical way to Unit Test
    // LCOV_EXCL_START
    if (diagnostic.level > Level::Type::WARN) {
        return false;
    }
    // LCOV_EXCL_STOP
    process->initialize(get_basenodename(),
                        get_nodename(),
                        get_hostname(),
                        DIAGNOSTIC_SYSTEM,
                        DIAGNOSTIC_SUBSYSTEM,
                        DIAGNOSTIC_COMPONENT,
                        logger);
    std::vector<eros_diagnostic::DiagnosticType> diagnostic_types;
    diagnostic_types.push_back(eros_diagnostic::DiagnosticType::SOFTWARE);
    diagnostic_types.push_back(eros_diagnostic::DiagnosticType::COMMUNICATIONS);
    diagnostic_types.push_back(eros_diagnostic::DiagnosticType::REMOTE_CONTROL);
    process->enable_diagnostics(diagnostic_types);
    process->finish_initialization();
    diagnostic = finish_initialization();
    // No Practical way to Unit Test
    // LCOV_EXCL_START
    if (diagnostic.level > Level::Type::WARN) {
        return false;
    }
    // LCOV_EXCL_STOP
    if (diagnostic.level < Level::Type::WARN) {
        diagnostic.type = eros_diagnostic::DiagnosticType::SOFTWARE;
        diagnostic.level = Level::Type::INFO;
        diagnostic.message = eros_diagnostic::Message::NOERROR;
        diagnostic.description = "Node Configured.  Initializing.";
        get_logger()->log_diagnostic(diagnostic);
    }
    logger->disable_consoleprint();  // Disabling as Teleop Node will use console window.
    status = init_screen();
    // No Practical way to Unit Test
    // LCOV_EXCL_START
    if (process->request_statechange(Node::State::RUNNING, true) == false) {
        logger->log_warn("Unable to Change State to: " +
                         Node::NodeStateString(Node::State::RUNNING));
    }
    // LCOV_EXCL_STOP
    logger->log_notice("Node State: " + Node::NodeStateString(process->get_nodestate()));
    status = true;
    return status;
}

eros_diagnostic::Diagnostic TeleopNode::read_launchparameters() {
    eros_diagnostic::Diagnostic diag = diagnostic;
    get_logger()->log_notice("Configuration Files Loaded.");
    return diag;
}
eros_diagnostic::Diagnostic TeleopNode::finish_initialization() {
    eros_diagnostic::Diagnostic diag = diagnostic;
    command_sub = n->subscribe<eros::command>(
        get_robotnamespace() + "SystemCommand", 10, &TeleopNode::command_Callback, this);
    std::string srv_nodestate_topic = "srv_nodestate_change";
    nodestate_srv =
        n->advertiseService(srv_nodestate_topic, &TeleopNode::changenodestate_service, this);
    diag = process->update_diagnostic(eros_diagnostic::DiagnosticType::COMMUNICATIONS,
                                      Level::Type::INFO,
                                      eros_diagnostic::Message::NOERROR,
                                      "Comms Ready.");
    diag = process->update_diagnostic(eros_diagnostic::DiagnosticType::SOFTWARE,
                                      Level::Type::INFO,
                                      eros_diagnostic::Message::NOERROR,
                                      "Running.");
    get_logger()->log_notice("Configuration Files Loaded.");
    return diag;
}

// Not used by Node, No way to unit test.  Still need to keep for BaseNode.
// LCOV_EXCL_START
bool TeleopNode::run_loop1() {
    return true;
}
// LCOV_EXCL_STOP
// Not used by Node, No way to unit test.  Still need to keep for BaseNode.
// LCOV_EXCL_START
bool TeleopNode::run_loop2() {
    return true;
}
// LCOV_EXCL_STOP
// Not used by Node, No way to unit test.  Still need to keep for BaseNode.
// LCOV_EXCL_START
bool TeleopNode::run_loop3() {
    return true;
}
// LCOV_EXCL_STOP
bool TeleopNode::run_001hz() {
    return true;
}
bool TeleopNode::run_01hz() {
    return true;
}
bool TeleopNode::run_01hz_noisy() {
    eros_diagnostic::Diagnostic diag = diagnostic;
    logger->log_debug(pretty());
    return true;
}
std::string TeleopNode::pretty() {
    std::string str = process->pretty();
    return str;
}
bool TeleopNode::run_1hz() {
    std::vector<eros_diagnostic::Diagnostic> latest_diagnostics = process->get_latest_diagnostics();
    for (std::size_t i = 0; i < latest_diagnostics.size(); ++i) {
        logger->log_diagnostic(latest_diagnostics.at(i));
        diagnostic_pub.publish(
            eros_diagnostic::DiagnosticUtility::convert(latest_diagnostics.at(i)));
    }
    eros_diagnostic::Diagnostic diag = process->get_root_diagnostic();
    if (process->get_nodestate() == Node::State::RESET) {
        base_reset();
        process->reset();
        logger->log_notice("Node has Reset");
        // No Practical way to Unit Test
        // LCOV_EXCL_START
        if (process->request_statechange(Node::State::RUNNING) == false) {
            diag = process->update_diagnostic(eros_diagnostic::DiagnosticType::SOFTWARE,
                                              Level::Type::ERROR,
                                              eros_diagnostic::Message::DEVICE_NOT_AVAILABLE,
                                              "Not able to Change Node State to "
                                              "Running.");
            logger->log_diagnostic(diag);
        }
        // LCOV_EXCL_STOP
    }
    return true;
}
bool TeleopNode::run_10hz() {
    process->update(0.1, ros::Time::now().toSec());
    if (process->get_killme() == true) {
        kill_node = true;
    }
    update_diagnostics(process->get_diagnostics());
    update_ready_to_arm(process->get_ready_to_arm());
    return true;
}
void TeleopNode::thread_loop() {
    while (kill_node == false) { ros::Duration(1.0).sleep(); }
}
// No Practical way to Unit Test
// LCOV_EXCL_START
void TeleopNode::cleanup() {
    process->request_statechange(Node::State::FINISHED);
    process->cleanup();
    logger->enable_consoleprint();
    logger->log_notice("Closing System Teleop Node.");
    base_cleanup();
}
// LCOV_EXCL_STOP

bool TeleopNode::init_screen() {
    setlocale(LC_ALL, "");
    mousemask(ALL_MOUSE_EVENTS, NULL);
    initscr();
    clear();
    if (has_colors() == FALSE) {
        endwin();
        logger->enable_consoleprint();
        logger->log_error("Terminal does not support colors. Exiting.");
        return false;
    }
    curs_set(0);
    noecho();
    raw();

    start_color();
    init_color(COLOR_BLACK, 0, 0, 0);
    init_color(COLOR_GREEN, 0, 600, 0);
    init_color(10, 500, 0, 500);
    init_pair((uint8_t)Color::NO_COLOR, COLOR_WHITE, COLOR_BLACK);
    init_pair((uint8_t)Color::RED_COLOR, COLOR_WHITE, COLOR_RED);
    init_pair((uint8_t)Color::YELLOW_COLOR, COLOR_WHITE, COLOR_YELLOW);
    init_pair((uint8_t)Color::GREEN_COLOR, COLOR_WHITE, COLOR_GREEN);
    init_pair((uint8_t)Color::BLUE_COLOR, COLOR_WHITE, COLOR_BLUE);
    init_pair((uint8_t)Color::PURPLE_COLOR, COLOR_WHITE, 10);

    uint16_t mainwindow_width, mainwindow_height;
    getmaxyx(stdscr, mainwindow_height, mainwindow_width);
    bool status = process->set_mainwindow(mainwindow_width, mainwindow_height);
    if (status == false) {
        logger->enable_consoleprint();
        logger->log_error("Window: Width: " + std::to_string(mainwindow_width) + " Height: " +
                          std::to_string(mainwindow_height) + " is too small. Exiting.");
        return false;
    }
    status = process->initialize_windows();
    if (status == false) {
        logger->enable_consoleprint();
        logger->log_error("Unable to initialize Windows. Exiting. ");
        return false;
    }
    return true;
}
void signalinterrupt_handler(int sig) {
    printf("Killing TeleopNode with Signal: %d\n", sig);
    kill_node = true;
    exit(0);
}
int main(int argc, char **argv) {
    signal(SIGINT, signalinterrupt_handler);
    signal(SIGTERM, signalinterrupt_handler);
    ros::init(argc, argv, "teleop_node");
    TeleopNode *node = new TeleopNode();
    bool status = node->start();
    // No Practical way to Unit Test
    // LCOV_EXCL_START
    if (status == false) {
        return EXIT_FAILURE;
    }
    // LCOV_EXCL_STOP
    std::thread thread(&TeleopNode::thread_loop, node);
    while ((status == true) and (kill_node == false)) {
        status = node->update(node->get_process()->get_nodestate());
    }

    // No Practical way to Unit Test
    // LCOV_EXCL_START
    thread.detach();
    node->cleanup();
    delete node;
    return 0;
    // LCOV_EXCL_STOP
}
