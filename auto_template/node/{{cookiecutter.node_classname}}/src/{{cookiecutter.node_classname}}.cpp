#include "{{cookiecutter.node_classname}}.h"
using namespace eros;
bool kill_node = false;
namespace {{cookiecutter.package_name}} {

{{cookiecutter.node_classname}}::{{cookiecutter.node_classname}}()
    : system_command_action_server(
          *n.get(),
          read_robotnamespace() + "SystemCommandAction",
          boost::bind(&{{cookiecutter.node_classname}}::system_commandAction_Callback, this, _1),
          false) {
    system_command_action_server.start();
}
{{cookiecutter.node_classname}}::~{{cookiecutter.node_classname}}() {
}
void {{cookiecutter.node_classname}}::system_commandAction_Callback(const eros::system_commandGoalConstPtr &goal) {
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
void {{cookiecutter.node_classname}}::command_Callback(const eros::command::ConstPtr &t_msg) {
    eros::command cmd = eros_utility::ConvertUtility::convert_fromptr(t_msg);
    eros_diagnostic::Diagnostic diag = process->get_root_diagnostic();
    diag = process->update_diagnostic(
        eros_diagnostic::DiagnosticType::COMMUNICATIONS,
        Level::Type::WARN,
        eros_diagnostic::Message::DROPPING_PACKETS,
        "Received unsupported Command: " + Command::CommandString((Command::Type)cmd.Command));
    logger->log_diagnostic(diag);
}
bool {{cookiecutter.node_classname}}::changenodestate_service(eros::srv_change_nodestate::Request &req,
                                         eros::srv_change_nodestate::Response &res) {
    Node::State req_state = Node::NodeState(req.RequestedNodeState);
    process->request_statechange(req_state);
    res.NodeState = Node::NodeStateString(process->get_nodestate());
    return true;
}
bool {{cookiecutter.node_classname}}::start() {
    initialize_diagnostic(DIAGNOSTIC_SYSTEM, DIAGNOSTIC_SUBSYSTEM, DIAGNOSTIC_COMPONENT);
    bool status = false;
    process = new {{cookiecutter.process_classname}}();
    set_basenodename(BASE_NODE_NAME);
    initialize_firmware(
        MAJOR_RELEASE_VERSION, MINOR_RELEASE_VERSION, BUILD_NUMBER, FIRMWARE_DESCRIPTION);
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
    // Add other Diagnostic Types as Needed
    process->enable_diagnostics(diagnostic_types);
    process->finish_initialization();
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
    if (process->request_statechange(Node::State::INITIALIZING) == false) {
        // No practical way to unit test
        // LCOV_EXCL_START
        logger->log_warn("Unable to Change State to: " +
                         Node::NodeStateString(Node::State::INITIALIZING));
        // LCOV_EXCL_STOP
    }
    if (process->request_statechange(Node::State::INITIALIZED) == false) {
        // No practical way to unit test
        // LCOV_EXCL_START
        logger->log_warn("Unable to Change State to: " +
                         Node::NodeStateString(Node::State::INITIALIZED));
        // LCOV_EXCL_STOP
    }
    if (process->request_statechange(Node::State::RUNNING) == false) {
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
eros_diagnostic::Diagnostic {{cookiecutter.node_classname}}::read_launchparameters() {
    eros_diagnostic::Diagnostic diag = diagnostic;
    command_sub = n->subscribe<eros::command>(
        get_robotnamespace() + "SystemCommand", 10, &SampleNode::command_Callback, this);
    get_logger()->log_notice("Configuration Files Loaded.");
    return diag;
}
eros_diagnostic::Diagnostic {{cookiecutter.node_classname}}::finish_initialization() {
    eros_diagnostic::Diagnostic diag = diagnostic;
    std::string srv_nodestate_topic = "srv_nodestate_change";
    nodestate_srv =
        n->advertiseService(srv_nodestate_topic, &{{cookiecutter.node_classname}}::changenodestate_service, this);
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
    return diag;
}
bool {{cookiecutter.node_classname}}::run_loop1() {
    return true;
}
bool {{cookiecutter.node_classname}}::run_loop2() {
    return true;
}
bool {{cookiecutter.node_classname}}::run_loop3() {
    return true;
}
bool {{cookiecutter.node_classname}}::run_001hz() {
    return true;
}
bool {{cookiecutter.node_classname}}::run_01hz() {
    return true;
}
bool {{cookiecutter.node_classname}}::run_01hz_noisy() {
    eros_diagnostic::Diagnostic diag = diagnostic;
    logger->log_notice("Node State: " + Node::NodeStateString(process->get_nodestate()));
    logger->log_debug(pretty());
    return true;
}
bool {{cookiecutter.node_classname}}::run_1hz() {
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
bool {{cookiecutter.node_classname}}::run_10hz() {
    update_diagnostics(process->get_diagnostics());
    update_ready_to_arm(process->get_ready_to_arm());
    return true;
}
void {{cookiecutter.node_classname}}::thread_loop() {
    while (kill_node == false) { ros::Duration(1.0).sleep(); }
}
std::string {{cookiecutter.node_classname}}::pretty() {
    std::string str = "--- {{cookiecutter.node_classname}} ---\n";
    str += process->pretty();
    return str;
}
void {{cookiecutter.node_classname}}::cleanup() {
    process->request_statechange(Node::State::FINISHED);
    process->cleanup();
    delete process;
    base_cleanup();
}
}  // namespace {{cookiecutter.package_name}}
// No practical way to unit test
// LCOV_EXCL_START
void signalinterrupt_handler(int sig) {
    printf("Killing {{cookiecutter.node_classname}} with Signal: %d\n", sig);
    kill_node = true;
    exit(0);
}
// LCOV_EXCL_STOP
int main(int argc, char **argv) {
    signal(SIGINT, signalinterrupt_handler);
    signal(SIGTERM, signalinterrupt_handler);
    ros::init(argc, argv, "{{cookiecutter.node_name_binary}}");
    {{cookiecutter.package_name}}::{{cookiecutter.node_classname}} *node = new {{cookiecutter.package_name}}::{{cookiecutter.node_classname}}();
    bool status = node->start();
    if (status == false) {
        // No practical way to unit test
        // LCOV_EXCL_START
        return EXIT_FAILURE;
        // LCOV_EXCL_STOP
    }
    std::thread thread(&{{cookiecutter.package_name}}::{{cookiecutter.node_classname}}::thread_loop, node);
    while ((status == true) and (kill_node == false)) {
        status = node->update(node->get_process()->get_nodestate());
    }
    node->cleanup();
    thread.detach();
    delete node;
    return 0;
}
