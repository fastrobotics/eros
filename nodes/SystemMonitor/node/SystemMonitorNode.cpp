#include "SystemMonitorNode.h"
using namespace eros;
bool kill_node = false;

namespace eros_nodes::SystemMonitor {

SystemMonitorNode::SystemMonitorNode()
    : system_command_action_server(
          *n.get(),
          extract_robotnamespace(n->getUnresolvedNamespace()) + "/SystemCommandAction",
          boost::bind(&SystemMonitorNode::system_commandAction_Callback, this, _1),
          false) {
    filter_list.insert(std::make_pair("rostopic", true));
    system_command_action_server.start();
}
SystemMonitorNode::~SystemMonitorNode() {
}
void SystemMonitorNode::command_Callback(const eros::command::ConstPtr &t_msg) {
    (void)t_msg;
}
void SystemMonitorNode::system_commandAction_Callback(
    const eros::system_commandGoalConstPtr &goal) {
    eros_diagnostic::Diagnostic diag = process->get_root_diagnostic();
    eros::system_commandResult result_;
    system_command_action_server.setAborted(result_);
    diag = process->update_diagnostic(
        eros_diagnostic::DiagnosticType::COMMUNICATIONS,
        Level::Type::WARN,
        eros_diagnostic::Message::DROPPING_PACKETS,
        "Received unsupported Command: " + Command::CommandString((Command::Type)goal->Command));
    logger->log_diagnostic(diag);
}
bool SystemMonitorNode::changenodestate_service(eros::srv_change_nodestate::Request &req,
                                                eros::srv_change_nodestate::Response &res) {
    Node::State req_state = Node::NodeState(req.RequestedNodeState);
    process->request_statechange(req_state);
    res.NodeState = Node::NodeStateString(process->get_nodestate());
    return true;
}
std::string SystemMonitorNode::extract_robotnamespace(std::string str) {
    std::string _robot_namespace;
    _robot_namespace = str.substr(0, str.find(BASE_NODE_NAME));
    return _robot_namespace;
}
bool SystemMonitorNode::start() {
    set_no_launch_enabled(true);

    initialize_diagnostic(DIAGNOSTIC_SYSTEM, DIAGNOSTIC_SUBSYSTEM, DIAGNOSTIC_COMPONENT);
    bool status = false;
    process = new SystemMonitorProcess();
    set_basenodename(BASE_NODE_NAME);
    initialize_firmware(
        MAJOR_RELEASE_VERSION, MINOR_RELEASE_VERSION, BUILD_NUMBER, FIRMWARE_DESCRIPTION);
    set_robotnamespace(ros::this_node::getNamespace());
    diagnostic = preinitialize_basenode();
    if (diagnostic.level > Level::Type::WARN) {
        return false;
    }
    if (diagnostic.level > Level::Type::WARN) {
        return false;
    }

    process->initialize(get_basenodename(),
                        get_nodename(),
                        get_hostname(),
                        DIAGNOSTIC_SYSTEM,
                        DIAGNOSTIC_SUBSYSTEM,
                        DIAGNOSTIC_COMPONENT,
                        logger);
    std::vector<eros_diagnostic::DiagnosticType> diagnostic_types;
    diagnostic_types.push_back(eros_diagnostic::DiagnosticType::SOFTWARE);
    diagnostic_types.push_back(eros_diagnostic::DiagnosticType::DATA_STORAGE);
    diagnostic_types.push_back(eros_diagnostic::DiagnosticType::SYSTEM_RESOURCE);
    diagnostic_types.push_back(eros_diagnostic::DiagnosticType::COMMUNICATIONS);
    process->enable_diagnostics(diagnostic_types);
    process->finish_initialization();
    process->set_nodeHandle((n.get()), get_robotnamespace());
    diagnostic = finish_initialization();
    logger->set_logverbosity(Level::Type::DEBUG);

    if (diagnostic.level > Level::Type::WARN) {
        return false;
    }
    if (diagnostic.level < Level::Type::WARN) {
        diagnostic.type = eros_diagnostic::DiagnosticType::SOFTWARE;
        diagnostic.level = Level::Type::INFO;
        diagnostic.message = eros_diagnostic::Message::NOERROR;
        diagnostic.description = "Node Configured.  Initializing.";
        get_logger()->log_diagnostic(diagnostic);
    }

    logger->disable_consoleprint();  // Disabling as System Monitor will use console window.
    status = init_screen();
    if (status == false) {
        logger->enable_consoleprint();
        logger->log_error("Unable to initialize screen");
    }
    diagnostic = rescan_nodes();
    if (diagnostic.level >= Level::Type::ERROR) {
        logger->log_diagnostic(diagnostic);
        return false;
    }
    if (process->request_statechange(Node::State::RUNNING, true) == false) {
        // No practical way to unit test
        // LCOV_EXCL_START
        logger->log_warn("Unable to Change State to: " +
                         Node::NodeStateString(Node::State::RUNNING));
        // LCOV_EXCL_STOP
    }
    logger->log_notice("Node State: " + Node::NodeStateString(process->get_nodestate()));
    return status;
}
eros_diagnostic::Diagnostic SystemMonitorNode::read_launchparameters() {
    eros_diagnostic::Diagnostic diag = diagnostic;
    get_logger()->log_notice("Configuration Files Loaded.");
    return diag;
}
eros_diagnostic::Diagnostic SystemMonitorNode::finish_initialization() {
    eros_diagnostic::Diagnostic diag = diagnostic;
    std::string srv_nodestate_topic = node_name + "/srv_nodestate_change";
    nodestate_srv =
        n->advertiseService(srv_nodestate_topic, &SystemMonitorNode::changenodestate_service, this);
    std::string commandstate_topic = "/" + get_robotnamespace() + "/SystemCommandState";
    commandstate_sub = n->subscribe<eros::command_state>(
        commandstate_topic, 50, &SystemMonitorNode::commandState_Callback, this);

    diag = process->update_diagnostic(eros_diagnostic::DiagnosticType::SOFTWARE,
                                      Level::Type::INFO,
                                      eros_diagnostic::Message::NOERROR,
                                      "Running");
    diag = process->update_diagnostic(eros_diagnostic::DiagnosticType::DATA_STORAGE,
                                      Level::Type::INFO,
                                      eros_diagnostic::Message::NOERROR,
                                      "All Configuration Files Loaded.");

    set_loop1_rate(1.0);
    set_loop2_rate(0.2);
    set_ros_rate(20.0);
    return diag;
}
bool SystemMonitorNode::run_loop1() {
    return true;
}
bool SystemMonitorNode::run_loop2() {
    eros_diagnostic::Diagnostic diag = rescan_nodes();
    if (diag.level > Level::Type::NOTICE) {
        logger->log_diagnostic(diag);
    }
    logger->log_info(process->pretty());
    return true;
}
bool SystemMonitorNode::run_loop3() {
    return true;
}
bool SystemMonitorNode::run_001hz() {
    return true;
}
bool SystemMonitorNode::run_01hz() {
    return true;
}
bool SystemMonitorNode::run_01hz_noisy() {
    eros_diagnostic::Diagnostic diag = diagnostic;
    logger->log_debug(pretty());
    return true;
}
std::string SystemMonitorNode::pretty() {
    std::string str = process->pretty();
    return str;
}
bool SystemMonitorNode::run_1hz() {
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
        if (process->request_statechange(Node::State::RUNNING) == false) {
            diag = process->update_diagnostic(eros_diagnostic::DiagnosticType::SOFTWARE,
                                              Level::Type::ERROR,
                                              eros_diagnostic::Message::DEVICE_NOT_AVAILABLE,
                                              "Not able to Change Node State to Running.");
            logger->log_diagnostic(diag);
        }
    }
    return true;
}
bool SystemMonitorNode::run_10hz() {
    if (process->get_killme() == true) {
        kill_node = true;
    }
    process->update_armedstate(eros_utility::ConvertUtility::convert(armed_state));
    eros_diagnostic::Diagnostic diagnostic = process->update(.1, ros::Time::now().toSec());
    if (diagnostic.level > Level::Type::NOTICE) {
        logger->log_diagnostic(diagnostic);
    }
    if (diagnostic.level > Level::Type::WARN) {
        return false;
    }
    return true;
}
void SystemMonitorNode::thread_loop() {
    while (kill_node == false) { ros::Duration(1.0).sleep(); }
}
void SystemMonitorNode::cleanup() {
    process->request_statechange(Node::State::FINISHED);
    process->cleanup();
    logger->enable_consoleprint();
    logger->log_notice("Closing System Monitor.");
    base_cleanup();
    delete process;
}

bool SystemMonitorNode::init_screen() {
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
    init_pair((uint8_t)eros_window::Color::NO_COLOR, COLOR_WHITE, COLOR_BLACK);
    init_pair((uint8_t)eros_window::Color::RED_COLOR, COLOR_WHITE, COLOR_RED);
    init_pair((uint8_t)eros_window::Color::YELLOW_COLOR, COLOR_WHITE, COLOR_YELLOW);
    init_pair((uint8_t)eros_window::Color::GREEN_COLOR, COLOR_WHITE, COLOR_GREEN);
    init_pair((uint8_t)eros_window::Color::BLUE_COLOR, COLOR_WHITE, COLOR_BLUE);
    init_pair((uint8_t)eros_window::Color::PURPLE_COLOR, COLOR_WHITE, 10);

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
void SystemMonitorNode::heartbeat_Callback(const eros::heartbeat::ConstPtr &t_msg) {
    eros_diagnostic::Diagnostic diag = process->new_heartbeatmessage(t_msg);
    if (diag.level > Level::Type::NOTICE) {
        logger->log_diagnostic(diag);
    }
}
void SystemMonitorNode::commandState_Callback(const eros::command_state::ConstPtr &t_msg) {
    eros_diagnostic::Diagnostic diag = process->new_commandstate(t_msg);
    if (diag.level > Level::Type::NOTICE) {
        logger->log_diagnostic(diag);
    }
}
void SystemMonitorNode::resourceused_Callback(const eros::resource::ConstPtr &t_msg) {
    eros_diagnostic::Diagnostic diag = process->new_resourceusedmessage(t_msg);
    if (diag.level > Level::Type::NOTICE) {
        logger->log_diagnostic(diag);
    }
}
void SystemMonitorNode::resourceavailable_Callback(const eros::resource::ConstPtr &t_msg) {
    eros_diagnostic::Diagnostic diag = process->new_resourceavailablemessage(t_msg);
    if (diag.level > Level::Type::NOTICE) {
        logger->log_diagnostic(diag);
    }
}
void SystemMonitorNode::loadfactor_Callback(const eros::loadfactor::ConstPtr &t_msg) {
    logger->log_warn(t_msg->DeviceName);
    eros_diagnostic::Diagnostic diag = process->new_loadfactormessage(t_msg);
    if (diag.level > Level::Type::NOTICE) {
        logger->log_diagnostic(diag);
    }
}
eros_diagnostic::Diagnostic SystemMonitorNode::rescan_nodes() {
    eros_diagnostic::Diagnostic diag = process->get_root_diagnostic();
    std::size_t found_new_subscribers = 0;
    ros::V_string nodes;
    ros::master::getNodes(nodes);
    std::vector<std::string> node_list;
    for (ros::V_string::iterator it = nodes.begin(); it != nodes.end(); it++) {
        const std::string &_node_name = *it;
        std::size_t found = _node_name.find(BASE_NODE_NAME);
        if (found != std::string::npos) {
            continue;
        }
        bool add_me = true;
        if (_node_name.rfind(get_robotnamespace(), 0) != 0) {
            add_me = false;
        }
        if (add_me == true) {
            std::map<std::string, bool>::iterator filter_it = filter_list.begin();
            while (filter_it != filter_list.end()) {
                if (filter_it->second == true) {
                    if (_node_name.find(filter_it->first) != std::string::npos) {
                        add_me = false;
                    }
                }
                filter_it++;
            }
        }
        if (add_me == true) {
            node_list.push_back(_node_name);
        }
    }
    ros::master::V_TopicInfo master_topics;
    ros::master::getTopics(master_topics);
    std::vector<std::string> heartbeat_list;
    std::vector<std::string> resource_used_list;
    std::vector<std::string> loadfactor_list;
    std::vector<std::string> resourceavailable_list;
    for (ros::master::V_TopicInfo::iterator it = master_topics.begin(); it != master_topics.end();
         it++) {
        const ros::master::TopicInfo &info = *it;
        std::size_t found = info.name.find(BASE_NODE_NAME);
        if (found != std::string::npos) {
            continue;
        }

        if (info.datatype == "eros/heartbeat") {
            if (info.name.rfind(get_robotnamespace(), 0) == 0) {
                heartbeat_list.push_back(info.name);
            }
        }
        if (info.datatype == "eros/loadfactor") {
            if (info.name.rfind(get_robotnamespace(), 0) == 0) {
                loadfactor_list.push_back(info.name);
            }
        }
        if (info.datatype == "eros/resource") {
            if (info.name.rfind(get_robotnamespace(), 0) == 0) {
                if (info.name.find("resource_used") != std::string::npos) {
                    resource_used_list.push_back(info.name);
                }
                else if (info.name.find("resource_available") != std::string::npos) {
                    resourceavailable_list.push_back(info.name);
                }
            }
        }
    }
    std::vector<std::string> new_heartbeat_topics_to_subscribe;
    std::vector<std::string> new_resourceused_topics_to_subscribe;
    std::vector<std::string> new_loadfactor_topics_to_subscribe;
    std::vector<std::string> new_resourceavailable_topics_to_subscribe;
    diag = process->update_monitorlist(heartbeat_list,
                                       resource_used_list,
                                       resourceavailable_list,
                                       loadfactor_list,
                                       new_heartbeat_topics_to_subscribe,
                                       new_resourceused_topics_to_subscribe,
                                       new_resourceavailable_topics_to_subscribe,
                                       new_loadfactor_topics_to_subscribe);
    if (diag.level >= Level::Type::WARN) {
        logger->log_diagnostic(diag);
        return diag;
    }
    for (std::size_t i = 0; i < new_heartbeat_topics_to_subscribe.size(); ++i) {
        logger->log_notice("Listening to new Topic: " + new_heartbeat_topics_to_subscribe.at(i));
        ros::Subscriber sub = n->subscribe<eros::heartbeat>(new_heartbeat_topics_to_subscribe.at(i),
                                                            50,
                                                            &SystemMonitorNode::heartbeat_Callback,
                                                            this);
        heartbeat_subs.push_back(sub);
    }
    for (std::size_t i = 0; i < new_resourceused_topics_to_subscribe.size(); ++i) {
        ros::Subscriber sub =
            n->subscribe<eros::resource>(new_resourceused_topics_to_subscribe.at(i),
                                         50,
                                         &SystemMonitorNode::resourceused_Callback,
                                         this);
        resource_used_subs.push_back(sub);
    }
    for (std::size_t i = 0; i < new_loadfactor_topics_to_subscribe.size(); ++i) {
        ros::Subscriber sub =
            n->subscribe<eros::loadfactor>(new_loadfactor_topics_to_subscribe.at(i),
                                           50,
                                           &SystemMonitorNode::loadfactor_Callback,
                                           this);
        loadfactor_subs.push_back(sub);
    }
    for (std::size_t i = 0; i < new_resourceavailable_topics_to_subscribe.size(); ++i) {
        ros::Subscriber sub =
            n->subscribe<eros::resource>(new_resourceavailable_topics_to_subscribe.at(i),
                                         50,
                                         &SystemMonitorNode::resourceavailable_Callback,
                                         this);
        resource_available_subs.push_back(sub);
    }
    found_new_subscribers = new_heartbeat_topics_to_subscribe.size() +
                            new_resourceused_topics_to_subscribe.size() +
                            new_resourceavailable_topics_to_subscribe.size() +
                            new_loadfactor_topics_to_subscribe.size();
    if (found_new_subscribers > 0) {
        logger->log_info("Rescanned and found " + std::to_string(found_new_subscribers) +
                         " new things to subscribe to.");
    }
    else {
        logger->log_info("Rescanned and found no new things to subscribe to.");
    }
    return diag;
}
}  // namespace eros_nodes::SystemMonitor
using namespace eros_nodes::SystemMonitor;
void signalinterrupt_handler(int sig) {
    printf("Killing SystemMonitorNode with Signal: %d\n", sig);
    kill_node = true;
    exit(0);
}
int main(int argc, char **argv) {
    signal(SIGINT, signalinterrupt_handler);
    signal(SIGTERM, signalinterrupt_handler);
    ros::init(argc, argv, "system_monitor");
    SystemMonitorNode *node = new SystemMonitorNode();
    bool status = node->start();
    if (status == false) {
        return EXIT_FAILURE;
    }
    std::thread thread(&SystemMonitorNode::thread_loop, node);
    while ((status == true) and (kill_node == false)) {
        status = node->update(node->get_process()->get_nodestate());
    }
    node->cleanup();
    thread.detach();
    delete node;
    return 0;
}
