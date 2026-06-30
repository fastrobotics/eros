#include "MasterNode.h"
using namespace eros;
using namespace eros_nodes::Infrastructure;
bool kill_node = false;
MasterNode::MasterNode()
    : system_command_action_server(
          *n.get(),
          read_robotnamespace() + "SystemCommandAction",
          boost::bind(&MasterNode::system_commandAction_Callback, this, _1),
          false) {
    system_command_action_server.start();
}
MasterNode::~MasterNode() {
}

void MasterNode::system_commandAction_Callback(const eros::system_commandGoalConstPtr &goal) {
    eros_diagnostic::Diagnostic diag = process->get_root_diagnostic();
    eros::system_commandResult system_commandResult_;
    system_command_action_server.setAborted(system_commandResult_);
    diag = process->update_diagnostic(eros_diagnostic::DiagnosticType::COMMUNICATIONS,
                                      Level::Type::WARN,
                                      eros_diagnostic::Message::DROPPING_PACKETS,
                                      "Received unsupported CommandAction: " +
                                          Command::CommandString((Command::Type)goal->Command));
    logger->log_diagnostic(diag);
}
void MasterNode::command_Callback(const eros::command::ConstPtr &t_msg) {
    eros::command cmd = eros_utility::ConvertUtility::convert_fromptr(t_msg);
    auto diag_list = process->new_commandmsg(cmd);
    for (auto diag : diag_list) { logger->log_diagnostic(diag); }
}
bool MasterNode::changenodestate_service(eros::srv_change_nodestate::Request &req,
                                         eros::srv_change_nodestate::Response &res) {
    Node::State req_state = Node::NodeState(req.RequestedNodeState);
    process->request_statechange(req_state);
    res.NodeState = Node::NodeStateString(process->get_nodestate());
    return true;
}
bool MasterNode::device_service(eros::srv_device::Request &req, eros::srv_device::Response &res) {
    (void)req;  // Currently Unused
    (void)res;
    if (deviceInfo.received == true) {
        return true;
    }
    else {
        // No practical way to unit test
        // LCOV_EXCL_START
        return false;
        // LCOV_EXCL_STOP
    }
}
bool MasterNode::start() {
    disable_device_client = true;
    initialize_diagnostic(DIAGNOSTIC_SYSTEM, DIAGNOSTIC_SUBSYSTEM, DIAGNOSTIC_COMPONENT);
    bool status = false;
    process = new MasterNodeProcess();
    set_basenodename(BASE_NODE_NAME);
    initialize_firmware(
        MAJOR_RELEASE_VERSION, MINOR_RELEASE_VERSION, BUILD_NUMBER, FIRMWARE_DESCRIPTION);
    enable_ready_to_arm_pub(true);
    diagnostic = preinitialize_basenode();
    if (diagnostic.level > Level::Type::WARN) {
        // No practical way to unit test
        // LCOV_EXCL_START
        return false;
        // LCOV_EXCL_STOP
    }
    diagnostic = read_launchparameters();
    if (diagnostic.level > Level::Type::WARN) {
        // No practical way to unit test
        // LCOV_EXCL_START
        return false;
        // LCOV_EXCL_STOP
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
    deviceInfo.received = true;

    diagnostic = finish_initialization();
    if (diagnostic.level > Level::Type::WARN) {
        // No practical way to unit test
        // LCOV_EXCL_START
        return false;
        // LCOV_EXCL_STOP
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
eros_diagnostic::Diagnostic MasterNode::read_launchparameters() {
    eros_diagnostic::Diagnostic diag = diagnostic;
    command_sub = n->subscribe<eros::command>(
        get_robotnamespace() + "SystemCommand", 10, &MasterNode::command_Callback, this);
    get_logger()->log_notice("Configuration Files Loaded.");
    return diag;
}
eros_diagnostic::Diagnostic MasterNode::finish_initialization() {
    eros_diagnostic::Diagnostic diag = diagnostic;
    std::string srv_nodestate_topic = "srv_nodestate_change";
    nodestate_srv =
        n->advertiseService(srv_nodestate_topic, &MasterNode::changenodestate_service, this);
    std::string srv_device_topic = "srv_device";
    device_server_srv = n->advertiseService(srv_device_topic, &MasterNode::device_service, this);

    std::string resource_available_topic =
        get_robotnamespace() + get_hostname() + "/resource_available";
    resource_available_pub = n->advertise<eros::resource>(resource_available_topic, 1);

    std::string device_loadfactor_topic = get_robotnamespace() + get_hostname() + "/loadfactor";
    loadfactor_pub = n->advertise<eros::loadfactor>(device_loadfactor_topic, 5);
    diag = process->update_diagnostic(eros_diagnostic::DiagnosticType::COMMUNICATIONS,
                                      Level::Type::INFO,
                                      eros_diagnostic::Message::NOERROR,
                                      "Comms Ready.");
    diag = process->update_diagnostic(eros_diagnostic::DiagnosticType::SOFTWARE,
                                      Level::Type::INFO,
                                      eros_diagnostic::Message::NOERROR,
                                      "Running");
    diag = process->update_diagnostic(eros_diagnostic::DiagnosticType::DATA_STORAGE,
                                      Level::Type::INFO,
                                      eros_diagnostic::Message::NOERROR,
                                      "All Configuration Files Loaded.");

    resource_available_monitor = new ResourceMonitor(
        get_robotnamespace() + get_hostname(), ResourceMonitor::Mode::DEVICE, diag, logger);
    diag = resource_available_monitor->init();
    if (diag.level > Level::Type::WARN) {
        // No practical way to unit test
        // LCOV_EXCL_START
        logger->log_diagnostic(diag);
        return diag;
        // LCOV_EXCL_STOP
    }
    return diag;
}
bool MasterNode::run_loop1() {
    return true;
}
bool MasterNode::run_loop2() {
    return true;
}
bool MasterNode::run_loop3() {
    return true;
}
bool MasterNode::run_001hz() {
    return true;
}
bool MasterNode::run_01hz() {
    return true;
}
bool MasterNode::run_01hz_noisy() {
    logger->log_debug(pretty());
    eros_diagnostic::Diagnostic diag = resource_available_monitor->update(10.0);
    logger->log_diagnostic(diag);
    if (diag.level <= Level::Type::WARN) {
        {
            eros::resource msg = eros_utility::ConvertUtility::convert(
                resource_available_monitor->get_resourceinfo());
            msg.Name = get_robotnamespace() + get_hostname();
            msg.stamp = ros::Time::now();
            resource_available_pub.publish(msg);
        }
        {
            eros::loadfactor msg = resource_available_monitor->get_load_factor();
            loadfactor_pub.publish(msg);
        }
    }
    return true;
}
std::string MasterNode::pretty() {
    std::string str = process->pretty();
    return str;
}
bool MasterNode::run_1hz() {
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
            // No practical way to unit test
            // LCOV_EXCL_START
            diag = process->update_diagnostic(eros_diagnostic::DiagnosticType::SOFTWARE,
                                              Level::Type::ERROR,
                                              eros_diagnostic::Message::DEVICE_NOT_AVAILABLE,
                                              "Not able to Change Node State to Running.");
            logger->log_diagnostic(diag);
            // LCOV_EXCL_STOP
        }
    }
    return true;
}
bool MasterNode::run_10hz() {
    eros_diagnostic::Diagnostic diag = process->update(0.1, ros::Time::now().toSec());
    if (diag.level >= Level::Type::NOTICE) {
        // No practical way to unit test
        // LCOV_EXCL_START
        logger->log_diagnostic(diag);
        // LCOV_EXCL_STOP
    }
    update_diagnostics(process->get_diagnostics());
    update_ready_to_arm(process->get_ready_to_arm());
    return true;
}
void MasterNode::thread_loop() {
    while (kill_node == false) { ros::Duration(1.0).sleep(); }
}
void MasterNode::cleanup() {
    process->request_statechange(Node::State::FINISHED);
    process->cleanup();
    delete process;
    base_cleanup();
}
// No practical way to unit test
// LCOV_EXCL_START
void signalinterrupt_handler(int sig) {
    printf("Killing MasterNode with Signal: %d\n", sig);
    kill_node = true;
    exit(0);
}
// LCOV_EXCL_STOP
int main(int argc, char **argv) {
    signal(SIGINT, signalinterrupt_handler);
    signal(SIGTERM, signalinterrupt_handler);
    ros::init(argc, argv, "master_node");

    MasterNode *node = new MasterNode();
    bool status = node->start();
    if (status == false) {
        // No practical way to unit test
        // LCOV_EXCL_START
        return EXIT_FAILURE;
        // LCOV_EXCL_STOP
    }
    std::thread thread(&MasterNode::thread_loop, node);
    while ((status == true) and (kill_node == false)) {
        status = node->update(node->get_process()->get_nodestate());
    }
    node->cleanup();
    thread.detach();
    delete node;
    return 0;
}
