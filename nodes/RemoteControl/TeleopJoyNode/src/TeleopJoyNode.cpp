#include "TeleopJoyNode.h"
using namespace eros;
using namespace eros_nodes::Infrastructure;
bool kill_node = false;

TeleopJoyNode::TeleopJoyNode()
    : system_command_action_server(
          *n.get(),
          read_robotnamespace() + "SystemCommandAction",
          boost::bind(&TeleopJoyNode::system_commandAction_Callback, this, _1),
          false) {
    system_command_action_server.start();
}
TeleopJoyNode::~TeleopJoyNode() {
}
void TeleopJoyNode::system_commandAction_Callback(const eros::system_commandGoalConstPtr &goal) {
    (void)goal;
    eros::system_commandResult system_commandResult_;
    system_command_action_server.setAborted(system_commandResult_);
}
void TeleopJoyNode::command_Callback(const eros::command::ConstPtr &t_msg) {
    eros::command cmd = eros_utility::ConvertUtility::convert_fromptr(t_msg);
    auto diag_list = process->new_commandmsg(cmd);
    for (auto diag : diag_list) { logger->log_diagnostic(diag); }
}
void TeleopJoyNode::joy_Callback(const sensor_msgs::Joy::ConstPtr &t_msg) {
    process->new_joymsg(eros_utility::ConvertUtility::convert_fromptr(t_msg));
}
bool TeleopJoyNode::changenodestate_service(eros::srv_change_nodestate::Request &req,
                                            eros::srv_change_nodestate::Response &res) {
    Node::State req_state = Node::NodeState(req.RequestedNodeState);
    process->request_statechange(req_state);
    res.NodeState = Node::NodeStateString(process->get_nodestate());
    return true;
}
bool TeleopJoyNode::start() {
    initialize_diagnostic(DIAGNOSTIC_SYSTEM, DIAGNOSTIC_SUBSYSTEM, DIAGNOSTIC_COMPONENT);
    bool status = false;
    process = new TeleopJoyNodeProcess();
    set_basenodename(BASE_NODE_NAME);
    initialize_firmware(
        MAJOR_RELEASE_VERSION, MINOR_RELEASE_VERSION, BUILD_NUMBER, FIRMWARE_DESCRIPTION);
    enable_ready_to_arm_pub(true);
    diagnostic = preinitialize_basenode();
    if (diagnostic.level > Level::Type::WARN) {
        return false;
    }
    diagnostic = read_launchparameters();
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
    diagnostic_types.push_back(eros_diagnostic::DiagnosticType::REMOTE_CONTROL);
    diagnostic_types.push_back(eros_diagnostic::DiagnosticType::COMMUNICATIONS);
    process->enable_diagnostics(diagnostic_types);
    process->finish_initialization();
    diagnostic = finish_initialization();
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
    if (process->request_statechange(Node::State::RUNNING, true) == false) {
        // No practical way to unit test
        // LCOV_EXCL_START
        logger->log_warn("Unable to Change State to: " +
                         Node::NodeStateString(Node::State::RUNNING));
        // LCOV_EXCL_STOP
    }
    logger->log_notice("Node State: " + Node::NodeStateString(process->get_nodestate()));
    status = true;
    return status;
}
eros_diagnostic::Diagnostic TeleopJoyNode::read_launchparameters() {
    eros_diagnostic::Diagnostic diag = diagnostic;
    get_logger()->log_notice("Configuration Files Loaded.");
    return diag;
}
eros_diagnostic::Diagnostic TeleopJoyNode::finish_initialization() {
    eros_diagnostic::Diagnostic diag = diagnostic;
    std::string srv_nodestate_topic = node_name + "/srv_nodestate_change";
    nodestate_srv =
        n->advertiseService(srv_nodestate_topic, &TeleopJoyNode::changenodestate_service, this);
    armedstate_pub = n->advertise<eros::armed_state>(get_robotnamespace() + "/ArmedState", 2);
    command_sub = n->subscribe<eros::command>(
        get_robotnamespace() + "SystemCommand", 10, &TeleopJoyNode::command_Callback, this);
    command_pub = n->advertise<eros::command>(get_robotnamespace() + "SystemCommand", 1);
    joy_sub = n->subscribe<sensor_msgs::Joy>("/joy", 10, &TeleopJoyNode::joy_Callback, this);
    std::string commandvel_per_topic = get_robotnamespace() + "cmd_vel_perc";
    cmd_vel_pub = n->advertise<geometry_msgs::Twist>(commandvel_per_topic, 2);
    diag = process->update_diagnostic(eros_diagnostic::DiagnosticType::SOFTWARE,
                                      Level::Type::INFO,
                                      eros_diagnostic::Message::NOERROR,
                                      "Running");

    diag = process->update_diagnostic(eros_diagnostic::DiagnosticType::COMMUNICATIONS,
                                      Level::Type::INFO,
                                      eros_diagnostic::Message::NOERROR,
                                      "No Comm Events Yet...");

    diag = process->update_diagnostic(eros_diagnostic::DiagnosticType::DATA_STORAGE,
                                      Level::Type::INFO,
                                      eros_diagnostic::Message::NOERROR,
                                      "All Configuration Files Loaded.");
    return diag;
}
bool TeleopJoyNode::run_loop1() {
    cmd_vel_pub.publish(process->get_cmd_twist());
    return true;
}
bool TeleopJoyNode::run_loop2() {
    return true;
}
bool TeleopJoyNode::run_loop3() {
    return true;
}
bool TeleopJoyNode::run_001hz() {
    return true;
}
bool TeleopJoyNode::run_01hz() {
    return true;
}
bool TeleopJoyNode::run_01hz_noisy() {
    eros_diagnostic::Diagnostic diag = diagnostic;
    logger->log_debug(pretty());
    return true;
}
std::string TeleopJoyNode::pretty() {
    std::string str = process->pretty();
    return str;
}
bool TeleopJoyNode::run_1hz() {
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
    logger->log_debug(process->pretty());
    return true;
}
bool TeleopJoyNode::run_10hz() {
    update_diagnostics(process->get_diagnostics());
    eros_diagnostic::Diagnostic diag = process->update(0.1, ros::Time::now().toSec());
    if (diag.level >= Level::Type::NOTICE) {
        logger->log_diagnostic(diag);
    }
    process->update_armedstate(eros_utility::ConvertUtility::convert(armed_state));
    update_ready_to_arm(process->get_ready_to_arm());
    auto current_command = process->get_current_command();
    if (current_command.new_command == true) {
        command_pub.publish(current_command.command);
    }
    return true;
}
void TeleopJoyNode::thread_loop() {
    while (kill_node == false) { ros::Duration(1.0).sleep(); }
}
void TeleopJoyNode::cleanup() {
    process->request_statechange(Node::State::FINISHED);
    process->cleanup();
    delete process;
    base_cleanup();
}
void signalinterrupt_handler(int sig) {
    printf("Killing TeleopJoyNode with Signal: %d\n", sig);
    kill_node = true;
    exit(0);
}
int main(int argc, char **argv) {
    signal(SIGINT, signalinterrupt_handler);
    signal(SIGTERM, signalinterrupt_handler);
    ros::init(argc, argv, "teleop_joy_node");
    TeleopJoyNode *node = new TeleopJoyNode();
    bool status = node->start();
    if (status == false) {
        return EXIT_FAILURE;
    }
    std::thread thread(&TeleopJoyNode::thread_loop, node);
    while ((status == true) and (kill_node == false)) {
        status = node->update(node->get_process()->get_nodestate());
    }
    node->cleanup();
    thread.detach();
    delete node;
    return 0;
}
