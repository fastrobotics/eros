#include "SafetyNode.h"
using namespace eros;
using namespace eros_nodes::Infrastructure;
bool kill_node = false;

SafetyNode::SafetyNode()
    : system_command_action_server(
          *n.get(),
          read_robotnamespace() + "SystemCommandAction",
          boost::bind(&SafetyNode::system_commandAction_Callback, this, _1),
          false) {
    system_command_action_server.start();
}
SafetyNode::~SafetyNode() {
}
void SafetyNode::system_commandAction_Callback(const eros::system_commandGoalConstPtr &goal) {
    (void)goal;
    eros::system_commandResult system_commandResult_;
    system_command_action_server.setAborted(system_commandResult_);
}
void SafetyNode::command_Callback(const eros::command::ConstPtr &t_msg) {
    eros::command cmd = eros_utility::ConvertUtility::convert_fromptr(t_msg);
    auto diag_list = process->new_commandmsg(cmd);
    for (auto diag : diag_list) { logger->log_diagnostic(diag); }
}
bool SafetyNode::changenodestate_service(eros::srv_change_nodestate::Request &req,
                                         eros::srv_change_nodestate::Response &res) {
    Node::State req_state = Node::NodeState(req.RequestedNodeState);
    process->request_statechange(req_state);
    res.NodeState = Node::NodeStateString(process->get_nodestate());
    return true;
}
bool SafetyNode::start() {
    initialize_diagnostic(DIAGNOSTIC_SYSTEM, DIAGNOSTIC_SUBSYSTEM, DIAGNOSTIC_COMPONENT);
    bool status = false;
    process = new SafetyNodeProcess();
    set_basenodename(BASE_NODE_NAME);
    initialize_firmware(
        MAJOR_RELEASE_VERSION, MINOR_RELEASE_VERSION, BUILD_NUMBER, FIRMWARE_DESCRIPTION);
    disable_armedstate_sub();
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
eros_diagnostic::Diagnostic SafetyNode::read_launchparameters() {
    eros_diagnostic::Diagnostic diag = diagnostic;
    get_logger()->log_notice("Configuration Files Loaded.");
    return diag;
}
eros_diagnostic::Diagnostic SafetyNode::finish_initialization() {
    eros_diagnostic::Diagnostic diag = diagnostic;
    std::string srv_nodestate_topic = node_name + "/srv_nodestate_change";
    nodestate_srv =
        n->advertiseService(srv_nodestate_topic, &SafetyNode::changenodestate_service, this);
    armedstate_pub = n->advertise<eros::armed_state>(get_robotnamespace() + "/ArmedState", 2);
    command_sub = n->subscribe<eros::command>(
        get_robotnamespace() + "SystemCommand", 10, &SafetyNode::command_Callback, this);
    diag = process->update_diagnostic(eros_diagnostic::DiagnosticType::SOFTWARE,
                                      Level::Type::INFO,
                                      eros_diagnostic::Message::NOERROR,
                                      "Running");

    diag = process->update_diagnostic(eros_diagnostic::DiagnosticType::COMMUNICATIONS,
                                      Level::Type::INFO,
                                      eros_diagnostic::Message::NOERROR,
                                      "No Comm Events Yet...");
    // Read Ready To Arm Topics
    std::vector<std::string> topics;

    uint16_t counter = 0;
    bool valid_arm_topic = true;
    while (valid_arm_topic == true) {
        char param_topic[512];
        sprintf(param_topic, "%s/ReadyToArm_Topic_%03d", node_name.c_str(), counter);
        std::string topic;
        if (n->getParam(param_topic, topic) == true) {
            logger->log_notice("Subscribing to ReadyToArm Topic: " + topic);
            topics.push_back(topic);
        }
        else {
            valid_arm_topic = false;
            break;
        }
        counter++;
    }
    diag = process->update_diagnostic(eros_diagnostic::DiagnosticType::DATA_STORAGE,
                                      Level::Type::INFO,
                                      eros_diagnostic::Message::NOERROR,
                                      "All Configuration Files Loaded.");
    if (process->set_ready_to_arm_signals(topics) == false) {
        diag = process->update_diagnostic(eros_diagnostic::DiagnosticType::REMOTE_CONTROL,
                                          Level::Type::ERROR,
                                          eros_diagnostic::Message::INITIALIZING_ERROR,
                                          "Unable to initialize all Ready To Arm Signals.");
    }
    for (auto topic : topics) {
        ros::Subscriber sub = n->subscribe<eros::ready_to_arm>(
            topic, 10, boost::bind(&SafetyNode::ReadyToArmCallback, this, _1, topic));
        ready_to_arm_subs.push_back(sub);
    }
    return diag;
}
void SafetyNode::ReadyToArmCallback(const eros::ready_to_arm::ConstPtr &msg,
                                    const std::string &topic_name) {
    if (process->new_message_readytoarm(
            topic_name, eros_utility::ConvertUtility::convert_fromptr(msg)) == false) {
        logger->log_warn("Unable to process Topic: " + topic_name);
    }
}
bool SafetyNode::run_loop1() {
    return true;
}
bool SafetyNode::run_loop2() {
    return true;
}
bool SafetyNode::run_loop3() {
    return true;
}
bool SafetyNode::run_001hz() {
    return true;
}
bool SafetyNode::run_01hz() {
    return true;
}
bool SafetyNode::run_01hz_noisy() {
    eros_diagnostic::Diagnostic diag = diagnostic;
    logger->log_debug(pretty());
    return true;
}
std::string SafetyNode::pretty() {
    std::string str = process->pretty();
    return str;
}
bool SafetyNode::run_1hz() {
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
    if (process->get_armed_state().armed_state == (uint8_t)ArmDisarm::Type::DISARMED_CANNOTARM) {
        auto cannotarm_reasons = process->get_cannotarm_reasons();
        for (auto reason : cannotarm_reasons) { logger->log_warn(reason); }
    }
    logger->log_debug(process->pretty());
    return true;
}
bool SafetyNode::run_10hz() {
    update_diagnostics(process->get_diagnostics());
    eros_diagnostic::Diagnostic diag = process->update(0.1, ros::Time::now().toSec());
    if (diag.level >= Level::Type::NOTICE) {
        logger->log_diagnostic(diag);
    }

    armedstate_pub.publish(process->get_armed_state());
    return true;
}
void SafetyNode::thread_loop() {
    while (kill_node == false) { ros::Duration(1.0).sleep(); }
}
void SafetyNode::cleanup() {
    process->request_statechange(Node::State::FINISHED);
    process->cleanup();
    delete process;
    base_cleanup();
}
void signalinterrupt_handler(int sig) {
    printf("Killing SafetyNode with Signal: %d\n", sig);
    kill_node = true;
    exit(0);
}
int main(int argc, char **argv) {
    signal(SIGINT, signalinterrupt_handler);
    signal(SIGTERM, signalinterrupt_handler);
    ros::init(argc, argv, "safety_node");
    SafetyNode *node = new SafetyNode();
    bool status = node->start();
    if (status == false) {
        return EXIT_FAILURE;
    }
    std::thread thread(&SafetyNode::thread_loop, node);
    while ((status == true) and (kill_node == false)) {
        status = node->update(node->get_process()->get_nodestate());
    }
    node->cleanup();
    thread.detach();
    delete node;
    return 0;
}
