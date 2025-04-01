#include "DataLoggerNode.h"
using namespace eros;
using namespace eros_nodes::Infrastructure;
bool kill_node = false;
// Coverage Suppress not working for this function.
DataLoggerNode::~DataLoggerNode() {
}
DataLoggerNode::DataLoggerNode()
    : system_command_action_server(
          *n.get(),
          read_robotnamespace() + "SystemCommandAction",
          boost::bind(&DataLoggerNode::system_commandAction_Callback, this, _1),
          false) {
    system_command_action_server.start();
}
void DataLoggerNode::system_commandAction_Callback(const eros::system_commandGoalConstPtr &goal) {
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
void DataLoggerNode::command_Callback(const eros::command::ConstPtr &t_msg) {
    eros::command cmd = eros_utility::ConvertUtility::convert_fromptr(t_msg);
    auto diag_list = process->new_commandmsg(cmd);
    for (auto diag : diag_list) { logger->log_diagnostic(diag); }
}
bool DataLoggerNode::changenodestate_service(eros::srv_change_nodestate::Request &req,
                                             eros::srv_change_nodestate::Response &res) {
    Node::State req_state = Node::NodeState(req.RequestedNodeState);
    process->request_statechange(req_state);
    res.NodeState = Node::NodeStateString(process->get_nodestate());
    return true;
}
bool DataLoggerNode::start() {
    initialize_diagnostic(DIAGNOSTIC_SYSTEM, DIAGNOSTIC_SUBSYSTEM, DIAGNOSTIC_COMPONENT);
    bool status = false;
    process = new DataLoggerProcess();
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
    diagnostic_types.push_back(eros_diagnostic::DiagnosticType::DATA_STORAGE);
    diagnostic_types.push_back(eros_diagnostic::DiagnosticType::SYSTEM_RESOURCE);
    diagnostic_types.push_back(eros_diagnostic::DiagnosticType::COMMUNICATIONS);
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

eros_diagnostic::Diagnostic DataLoggerNode::read_launchparameters() {
    eros_diagnostic::Diagnostic diag = diagnostic;
    get_logger()->log_notice("Configuration Files Loaded.");
    return diag;
}
eros_diagnostic::Diagnostic DataLoggerNode::finish_initialization() {
    eros_diagnostic::Diagnostic diag = diagnostic;
    std::string param_logfile_duration = node_name + "/LogFile_Duration";
    double logfile_duration;
    // No Practical way to Unit Test
    // LCOV_EXCL_START
    if (n->getParam(param_logfile_duration, logfile_duration) == false) {
        diag = process->update_diagnostic(eros_diagnostic::DiagnosticType::DATA_STORAGE,
                                          Level::Type::ERROR,
                                          eros_diagnostic::Message::INITIALIZING_ERROR,
                                          "Missing Parameter: LogFile_Duration.  Exiting.");
        logger->log_diagnostic(diag);
        return diag;
    }
    // LCOV_EXCL_STOP
    std::string param_logfile_directory = node_name + "/LogFile_Directory";
    std::string logfile_directory;
    // No Practical way to Unit Test
    // LCOV_EXCL_START
    if (n->getParam(param_logfile_directory, logfile_directory) == false) {
        diag = process->update_diagnostic(eros_diagnostic::DiagnosticType::DATA_STORAGE,
                                          Level::Type::ERROR,
                                          eros_diagnostic::Message::INITIALIZING_ERROR,
                                          "Missing Parameter: LogFile_Directory.  Exiting.");
        logger->log_diagnostic(diag);
        return diag;
    }
    // LCOV_EXCL_STOP
    process->set_logfileduration(logfile_duration);
    bool available = process->set_logdirectory(logfile_directory);
    // No Practical way to Unit Test
    // LCOV_EXCL_START
    if (available == false) {
        diag = process->update_diagnostic(
            eros_diagnostic::DiagnosticType::DATA_STORAGE,
            Level::Type::ERROR,
            eros_diagnostic::Message::DEVICE_NOT_AVAILABLE,
            "LogFile_Directory: " + logfile_directory + " Does Not Exist. Exiting.");
        logger->log_diagnostic(diag);
        return diag;
    }
    // LCOV_EXCL_STOP
    std::string param_snapshot_mode = node_name + "/SnapshotMode";
    bool snapshot_mode = false;
    // No Practical way to Unit Test
    // LCOV_EXCL_START
    if (n->getParam(param_snapshot_mode, snapshot_mode) == false) {
        diag = process->update_diagnostic(eros_diagnostic::DiagnosticType::DATA_STORAGE,
                                          Level::Type::WARN,
                                          eros_diagnostic::Message::NOERROR,
                                          "Missing Parameter: SnapshotMode.");
        logger->log_diagnostic(diag);
    }
    // LCOV_EXCL_STOP
    process->setSnapshotMode(snapshot_mode);
    // No Practical way to Unit Test
    // LCOV_EXCL_START
    if (snapshot_mode == false) {
        diag = process->update_diagnostic(eros_diagnostic::DiagnosticType::DATA_STORAGE,
                                          Level::Type::WARN,
                                          eros_diagnostic::Message::NOERROR,
                                          "SnapshotMode Disabled.  Logging to File Storage.");
        logger->log_diagnostic(diag);
    }
    // LCOV_EXCL_STOP
    else {
        snapshot_trigger_sub =
            n->subscribe<std_msgs::Empty>(get_robotnamespace() + "snapshot_trigger",
                                          1,
                                          &DataLoggerNode::snapshot_trigger_Callback,
                                          this);
        diag = process->update_diagnostic(eros_diagnostic::DiagnosticType::DATA_STORAGE,
                                          Level::Type::NOTICE,
                                          eros_diagnostic::Message::NOERROR,
                                          "SnapshotMode Enabled.  All logs stored in RAM until "
                                          "Snapshot is triggered.");
        logger->log_diagnostic(diag);
    }
    command_sub = n->subscribe<eros::command>(
        get_robotnamespace() + "SystemCommand", 10, &DataLoggerNode::command_Callback, this);
    std::string srv_nodestate_topic = "srv_nodestate_change";
    nodestate_srv =
        n->advertiseService(srv_nodestate_topic, &DataLoggerNode::changenodestate_service, this);
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
void DataLoggerNode::snapshot_trigger_Callback(const std_msgs::Empty::ConstPtr &t_msg) {
    (void)t_msg;
    logger->log_notice("Bag File Snapshot Trigger Received.");
    if (create_snapshot_file() == false) {
        logger->log_warn("Unable to create Snapshot File!");
    }
}
bool DataLoggerNode::create_snapshot_file() {
    std::ofstream snapshot_file;
    std::string snapshot_file_path = process->get_logdirectory() + "/BagFile_Snapshots.txt";
    snapshot_file.open(snapshot_file_path, std::ios::out | std::ios::app);
    if (snapshot_file.is_open() == true) {
        time_t rawtime;
        struct tm *timeinfo;
        char datebuffer[80];
        char bagFilebuffer[80];
        time(&rawtime);
        timeinfo = localtime(&rawtime);
        strftime(datebuffer, 80, "%b-%d-%Y %I:%M:%S", timeinfo);
        std::string time_str(datebuffer);
        strftime(bagFilebuffer, 80, "BAG_%Y-%m-%d-%I-%M-%S_0.bag", timeinfo);
        std::string probable_bag_file(bagFilebuffer);
        snapshot_file << "-----------" << std::endl;
        snapshot_file << "BagFile Snapshot Requested at: " << time_str << std::endl;
        snapshot_file << "Time: " << std::to_string(ros::Time::now().toSec()) << " (Unix Time)"
                      << std::endl;
        snapshot_file << "Probable Bag File Name: " << probable_bag_file
                      << " (Time may not be exactly right...)" << std::endl;
        snapshot_file.close();
    }
    // No Practical way to Unit Test
    // LCOV_EXCL_START
    else {
        logger->log_warn("Could not open file: " + snapshot_file_path);
        return false;
    }
    // LCOV_EXCL_STOP
    return true;
}
// Not used by Node, No way to unit test.  Still need to keep for BaseNode.
// LCOV_EXCL_START
bool DataLoggerNode::run_loop1() {
    return true;
}
// LCOV_EXCL_STOP
// Not used by Node, No way to unit test.  Still need to keep for BaseNode.
// LCOV_EXCL_START
bool DataLoggerNode::run_loop2() {
    return true;
}
// LCOV_EXCL_STOP
// Not used by Node, No way to unit test.  Still need to keep for BaseNode.
// LCOV_EXCL_START
bool DataLoggerNode::run_loop3() {
    return true;
}
// LCOV_EXCL_STOP
bool DataLoggerNode::run_001hz() {
    return true;
}
bool DataLoggerNode::run_01hz() {
    return true;
}
bool DataLoggerNode::run_01hz_noisy() {
    eros_diagnostic::Diagnostic diag = diagnostic;
    logger->log_debug(pretty());
    return true;
}
std::string DataLoggerNode::pretty() {
    std::string str = process->pretty();
    return str;
}
bool DataLoggerNode::run_1hz() {
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
bool DataLoggerNode::run_10hz() {
    process->update(0.1, ros::Time::now().toSec());
    update_diagnostics(process->get_diagnostics());
    update_ready_to_arm(process->get_ready_to_arm());
    return true;
}
void DataLoggerNode::thread_loop() {
    while (kill_node == false) { ros::Duration(1.0).sleep(); }
}
void DataLoggerNode::run_logger(DataLoggerNode *node) {
    if (node->get_process()->is_logging_enabled() == true) {
        rosbag::RecorderOptions opts;
        opts.record_all = false;
        opts.quiet = true;
        opts.verbose = false;
        opts.prefix = node->get_process()->get_logdirectory() + "BAG";
        opts.append_date = true;
        opts.max_duration =
            ros::Duration(node->get_process()->get_logfile_duration());  // 30 minutes
        opts.split = true;
        opts.snapshot = node->get_process()->getSnapshotMode();
        ros::master::V_TopicInfo master_topics;
        ros::master::getTopics(master_topics);
        for (std::size_t i = 0; i < master_topics.size(); ++i) {
            // No Practical way to Unit Test
            // LCOV_EXCL_START
            if (get_robotnamespace() == "/") {
                opts.topics.push_back(master_topics.at(i).name);
            }
            // LCOV_EXCL_STOP
            else if (master_topics.at(i).name.rfind(get_robotnamespace(), 0) == 0) {
                opts.topics.push_back(master_topics.at(i).name);
            }
        }
        rosbag::Recorder recorder(opts);
        logger->log_warn("Data Logger Running.");
        recorder.run();
        // No Practical way to Unit Test
        // LCOV_EXCL_START
        node->get_logger()->log_notice("Logger Finished.");
        // LCOV_EXCL_STOP
    }
    // No Practical way to Unit Test
    // LCOV_EXCL_START
    return;
    // LCOV_EXCL_STOP
}
// No Practical way to Unit Test
// LCOV_EXCL_START
void DataLoggerNode::cleanup() {
    process->request_statechange(Node::State::FINISHED);
    process->cleanup();
    delete process;
    base_cleanup();
}
// LCOV_EXCL_STOP
void signalinterrupt_handler(int sig) {
    printf("Killing DataLoggerNode with Signal: %d\n", sig);
    kill_node = true;
    exit(0);
}
int main(int argc, char **argv) {
    signal(SIGINT, signalinterrupt_handler);
    signal(SIGTERM, signalinterrupt_handler);
    ros::init(argc, argv, "datalogger_node");
    DataLoggerNode *node = new DataLoggerNode();
    bool status = node->start();
    // No Practical way to Unit Test
    // LCOV_EXCL_START
    if (status == false) {
        return EXIT_FAILURE;
    }
    // LCOV_EXCL_STOP
    std::thread thread(&DataLoggerNode::thread_loop, node);
    std::thread thread2(&DataLoggerNode::run_logger, node, node);
    while ((status == true) and (kill_node == false)) {
        status = node->update(node->get_process()->get_nodestate());
    }

    thread2.join();
    // No Practical way to Unit Test
    // LCOV_EXCL_START
    thread.detach();
    node->cleanup();
    delete node;
    return 0;
    // LCOV_EXCL_STOP
}
