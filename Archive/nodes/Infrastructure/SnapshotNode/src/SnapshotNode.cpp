#include "SnapshotNode.h"
using namespace eros;
using namespace eros_nodes::Infrastructure;
bool kill_node = false;
SnapshotNode::SnapshotNode()
    : system_command_action_server(
          *n.get(),
          read_robotnamespace() + "SystemCommandAction",
          boost::bind(&SnapshotNode::system_commandAction_Callback, this, _1),
          false) {
    system_command_action_server.start();
}
SnapshotNode::~SnapshotNode() {
}
void SnapshotNode::command_Callback(const eros::command::ConstPtr &t_msg) {
    eros_diagnostic::Diagnostic diag = process->get_root_diagnostic();
    eros::command command = eros_utility::ConvertUtility::convert_fromptr(t_msg);
    std::vector<eros_diagnostic::Diagnostic> diag_list = process->new_commandmsg(command);
    bool any_error = false;
    for (std::size_t i = 0; i < diag_list.size(); ++i) {
        if (diag_list.at(i).level >= Level::Type::WARN) {
            any_error = true;
            diag = diag_list.at(i);
            logger->log_diagnostic(diag);
        }
    }
    if (any_error == false) {
        diag.level = Level::Type::INFO;
        diag.message = eros_diagnostic::Message::NOERROR;
        diag.description = "No Error";
    }
    bool send_reply = false;
    if (t_msg->Command == (uint16_t)Command::Type::GENERATE_SNAPSHOT) {
        if (t_msg->Option1 == (uint16_t)Command::GenerateSnapshot_Option1::CLEAR_SNAPSHOTS) {
            send_reply = true;
        }
    }
    if (send_reply == true) {
        eros::command_state state;
        state.stamp = ros::Time::now();
        state.Name = get_hostname();
        state.CurrentCommand = command;
        state.State = 0;
        state.PercentComplete = 0.0;
        state.diag = eros_diagnostic::DiagnosticUtility::convert(diag);
        if (diag.level > Level::Type::NOTICE) {
            logger->log_diagnostic(diag);
        }
        commandstate_pub.publish(state);
    }
}
void SnapshotNode::commandState_Callback(const eros::command_state::ConstPtr &t_msg) {
    process->new_commandstatemsg(eros_utility::ConvertUtility::convert_fromptr(t_msg));
}
void SnapshotNode::system_commandAction_Callback(const eros::system_commandGoalConstPtr &goal) {
    (void)goal;
}
bool SnapshotNode::changenodestate_service(eros::srv_change_nodestate::Request &req,
                                           eros::srv_change_nodestate::Response &res) {
    Node::State req_state = Node::NodeState(req.RequestedNodeState);
    process->request_statechange(req_state);
    res.NodeState = Node::NodeStateString(process->get_nodestate());
    return true;
}
bool SnapshotNode::filetransfer_service(eros::srv_filetransfer::Request &req,
                                        eros::srv_filetransfer::Response &res) {
    res.files.clear();
    if (req.path == "") {
        return true;
    }
    else if ((req.path.at(req.path.length() - 1) == '/') ||
             (req.path.at(req.path.length() - 1) == '*'))  // Add a bunch of files
    {
        std::string path = BaseNodeProcess::sanitize_path(req.path);

        std::vector<std::string> files = process->get_files_indir(path);
        for (std::size_t i = 0; i < files.size(); ++i) {
            file f;
            FileHelper::FileInfo info = BaseNodeProcess::read_file(req.path + files.at(i));
            if (info.fileStatus == FileHelper::FileStatus::FILE_OK) {
                std::vector<uint8_t> vec(info.data, info.data + info.byte_size);
                f.file_name = info.file_name;
                f.data = vec;
                f.status = (uint8_t)FileHelper::FileStatus::FILE_OK;
                f.extension_type = (uint8_t)info.fileType;
                f.data_length = info.byte_size;
                res.files.push_back(f);
            }
            else {
                f.status = (uint8_t)FileHelper::FileStatus::FILE_ERROR;
                res.files.push_back(f);
            }
        }
    }
    else  // Single file
    {
        file f;
        FileHelper::FileInfo info = BaseNodeProcess::read_file(req.path);
        if (info.fileStatus == FileHelper::FileStatus::FILE_OK) {
            std::vector<uint8_t> vec(info.data, info.data + info.byte_size);
            f.data = vec;
            f.file_name = info.file_name;
            f.status = (uint8_t)FileHelper::FileStatus::FILE_OK;
            f.extension_type = (uint8_t)info.fileType;
            f.data_length = info.byte_size;
            res.files.push_back(f);
        }
        else {
            f.status = (uint8_t)FileHelper::FileStatus::FILE_ERROR;
            res.files.push_back(f);
            logger->log_error("Not able to read file at path: " + req.path);
        }
    }
    return true;
}
bool SnapshotNode::start() {
    initialize_diagnostic(DIAGNOSTIC_SYSTEM, DIAGNOSTIC_SUBSYSTEM, DIAGNOSTIC_COMPONENT);
    bool status = false;
    process = new SnapshotProcess();
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
    diagnostic_types.push_back(eros_diagnostic::DiagnosticType::COMMUNICATIONS);
    process->enable_diagnostics(diagnostic_types);
    process->set_architecture(resource_monitor->get_architecture());
    diagnostic = process->finish_initialization();
    if (diagnostic.level > Level::Type::WARN) {
        return false;
    }
    process->set_nodeHandle((n.get()), get_robotnamespace());
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
eros_diagnostic::Diagnostic SnapshotNode::read_launchparameters() {
    eros_diagnostic::Diagnostic diag = diagnostic;
    std::string param_mode = node_name + "/Mode";
    std::string mode;
    if (n->getParam(param_mode, mode) == false) {
        diag = process->update_diagnostic(eros_diagnostic::DiagnosticType::DATA_STORAGE,
                                          Level::Type::ERROR,
                                          eros_diagnostic::Message::INITIALIZING_ERROR,
                                          "Mode Not defined.");
        logger->log_diagnostic(diag);
        return diag;
    }
    process->set_mode(SnapshotProcess::ModeType(mode));
    get_logger()->log_notice("Snapshot Mode: " + SnapshotProcess::ModeString(process->get_mode()));
    get_logger()->log_notice("Configuration Files Loaded.");
    return diag;
}
eros_diagnostic::Diagnostic SnapshotNode::finish_initialization() {
    eros_diagnostic::Diagnostic diag = diagnostic;
    std::string srv_nodestate_topic = node_name + "/srv_nodestate_change";
    nodestate_srv =
        n->advertiseService(srv_nodestate_topic, &SnapshotNode::changenodestate_service, this);
    diag = process->update_diagnostic(eros_diagnostic::DiagnosticType::SOFTWARE,
                                      Level::Type::INFO,
                                      eros_diagnostic::Message::NOERROR,
                                      "Running");
    std::string param_override_devicenames = node_name + "/Override_DeviceNames";
    std::string override_devicename;
    std::vector<std::string> override_devicenames;
    if (n->getParam(param_override_devicenames, override_devicename) == false) {
        override_devicename = "";
    }
    else {
        override_devicenames.push_back(override_devicename);
    }
    std::string param_config_dir = node_name + "/Config_Directory";
    std::string config_dir;
    if (n->getParam(param_config_dir, config_dir) == false) {
        diag = process->update_diagnostic(eros_diagnostic::DiagnosticType::DATA_STORAGE,
                                          Level::Type::ERROR,
                                          eros_diagnostic::Message::INITIALIZING_ERROR,
                                          "Config_Directory Not defined.");
        logger->log_diagnostic(diag);
        return diag;
    }
    command_sub = n->subscribe<eros::command>(
        get_robotnamespace() + "SystemCommand", 10, &SnapshotNode::command_Callback, this);
    if (process->get_mode() == SnapshotProcess::Mode::MASTER) {
        commandstate_sub =
            n->subscribe<eros::command_state>(get_robotnamespace() + "SystemCommandState",
                                              10,
                                              &SnapshotNode::commandState_Callback,
                                              this);
    }
    else {
        std::string srv_filetransfer_topic = node_name + "/srv_filetransfer";
        filetransfer_srv =
            n->advertiseService(srv_filetransfer_topic, &SnapshotNode::filetransfer_service, this);
    }
    bagfile_snapshottrigger_pub =
        n->advertise<std_msgs::Empty>(get_robotnamespace() + "snapshot_trigger", 5);
    diag = process->load_config(config_dir + "/SnapshotConfig.xml", override_devicenames);
    if (diag.level >= Level::Type::ERROR) {
        logger->log_diagnostic(diag);
        return diag;
    }
    std::string commandstate_topic = get_robotnamespace() + "SystemCommandState";
    commandstate_pub = n->advertise<eros::command_state>(commandstate_topic, 1);
    std::string command_topic = get_robotnamespace() + "SystemCommand";
    command_pub = n->advertise<eros::command>(command_topic, 10);
    diag = process->update_diagnostic(eros_diagnostic::DiagnosticType::COMMUNICATIONS,
                                      Level::Type::INFO,
                                      eros_diagnostic::Message::NOERROR,
                                      "Running");
    diag = process->update_diagnostic(eros_diagnostic::DiagnosticType::DATA_STORAGE,
                                      Level::Type::INFO,
                                      eros_diagnostic::Message::NOERROR,
                                      "All Configuration Files Loaded.");
    return diag;
}
bool SnapshotNode::run_loop1() {
    return true;
}
bool SnapshotNode::run_loop2() {
    return true;
}
bool SnapshotNode::run_loop3() {
    return true;
}
bool SnapshotNode::run_001hz() {
    return true;
}
bool SnapshotNode::run_01hz() {
    return true;
}
bool SnapshotNode::run_01hz_noisy() {
    eros_diagnostic::Diagnostic diag = diagnostic;
    logger->log_debug(pretty());
    return true;
}
std::string SnapshotNode::pretty() {
    std::string str = process->pretty();
    return str;
}
bool SnapshotNode::run_1hz() {
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

    if (process->get_mode() == SnapshotProcess::Mode::MASTER) {
        if ((process->get_systemsnapshot_state() == eros::SnapshotState::COMPLETE) ||
            (process->get_systemsnapshot_state() == eros::SnapshotState::INCOMPLETE)) {
            process->set_systemsnapshot_state(eros::SnapshotState::NOTRUNNING);
        }
    }
    return true;
}
bool SnapshotNode::run_10hz() {
    eros_diagnostic::Diagnostic diag = process->update(0.1, ros::Time::now().toSec());
    if (diag.level >= Level::Type::NOTICE) {
        logger->log_diagnostic(diag);
    }
    update_diagnostics(process->get_diagnostics());
    update_ready_to_arm(process->get_ready_to_arm());
    if ((process->get_devicesnapshot_state() == eros::SnapshotState::RUNNING) ||
        (process->get_systemsnapshot_state() == eros::SnapshotState::RUNNING)) {
        eros::command_state state;
        state.stamp = ros::Time::now();
        state.CurrentCommand.Command = (uint16_t)Command::Type::GENERATE_SNAPSHOT;
        if (process->get_mode() == SnapshotProcess::Mode::MASTER) {
            state.CurrentCommand.Option1 = (uint16_t)Command::GenerateSnapshot_Option1::RUN_MASTER;
            state.State = (uint8_t)process->get_systemsnapshot_state();
        }
        else if (process->get_mode() == SnapshotProcess::Mode::SLAVE) {
            state.CurrentCommand.Option1 = (uint16_t)Command::GenerateSnapshot_Option1::RUN_SLAVE;
            state.State = (uint8_t)process->get_devicesnapshot_state();
        }
        state.Name = get_hostname();
        state.diag.Level = (uint8_t)Level::Type::DEBUG;
        state.diag.DiagnosticMessage = (uint8_t)eros_diagnostic::Message::NOERROR;
        state.PercentComplete = process->get_snapshotprogress_percentage();
        if (state.diag.Level > (uint8_t)Level::Type::NOTICE) {
            logger->log_diagnostic(eros_diagnostic::DiagnosticUtility::convert(state.diag));
        }
        commandstate_pub.publish(state);
    }
    return true;
}
void SnapshotNode::thread_loop() {
    while (kill_node == false) { ros::Duration(1.0).sleep(); }
}
void SnapshotNode::thread_snapshotcreation() {
    while (kill_node == false) {
        if (process->get_devicesnapshot_state() == eros::SnapshotState::STARTED) {
            process->set_devicesnapshot_state(eros::SnapshotState::RUNNING);

            logger->log_notice("Snap: " +
                               eros::SnapshotStateString(process->get_devicesnapshot_state()));
            SnapshotProcess::SnapshotConfig config = process->get_snapshot_config();
            eros_diagnostic::Diagnostic diag = process->get_root_diagnostic();
            if (process->get_mode() == SnapshotProcess::Mode::MASTER) {
                std_msgs::Empty bagfile_snapshot_trigger;
                bagfile_snapshottrigger_pub.publish(bagfile_snapshot_trigger);
                process->set_systemsnapshot_state(eros::SnapshotState::RUNNING);
                // Send Command to Slaves to Start
                eros::command command;
                command.stamp = ros::Time::now();
                command.Command = (uint16_t)Command::Type::GENERATE_SNAPSHOT;
                command.Option1 = (uint16_t)Command::GenerateSnapshot_Option1::RUN_SLAVE;
                for (std::size_t i = 0; i < 1; ++i) { command_pub.publish(command); }
            }
            std::vector<eros_diagnostic::Diagnostic> diag_list = process->createnew_snapshot();
            Level::Type max_level = Level::Type::DEBUG;
            for (std::size_t i = 0; i < diag_list.size(); ++i) {
                if (diag_list.at(i).level > max_level) {
                    max_level = diag_list.at(i).level;
                    diag = diag_list.at(i);
                }
            }
            if (max_level <= Level::Type::WARN) {
                logger->log_notice("Snap Completed");
                eros::command_state state;
                state.stamp = ros::Time::now();
                state.Name = get_hostname();
                state.CurrentCommand.Command = (uint16_t)Command::Type::GENERATE_SNAPSHOT;
                if (process->get_mode() == SnapshotProcess::Mode::MASTER) {
                    state.CurrentCommand.Option1 =
                        (uint16_t)Command::GenerateSnapshot_Option1::RUN_MASTER;
                    state.CurrentCommand.CommandText = "System Snap Completed.";
                    state.State = (uint8_t)process->get_systemsnapshot_state();
                }
                else if (process->get_mode() == SnapshotProcess::Mode::SLAVE) {
                    state.CurrentCommand.Option1 =
                        (uint16_t)Command::GenerateSnapshot_Option1::RUN_SLAVE;
                    state.CurrentCommand.CommandText =
                        process->get_snapshot_config().active_device_snapshot_completepath;
                    state.State = (uint8_t)process->get_devicesnapshot_state();
                }
                state.PercentComplete = 100.0;
                state.diag = eros_diagnostic::DiagnosticUtility::convert(diag);
                if (state.diag.Level > (uint8_t)Level::Type::NOTICE) {
                    logger->log_diagnostic(diag);
                }
                commandstate_pub.publish(state);
            }
            else {
                logger->log_warn("Snap Failed");
                eros::command_state state;
                state.stamp = ros::Time::now();
                state.Name = get_hostname();
                state.CurrentCommand.Command = (uint16_t)Command::Type::GENERATE_SNAPSHOT;
                if (process->get_mode() == SnapshotProcess::Mode::MASTER) {
                    state.CurrentCommand.Option1 =
                        (uint16_t)Command::GenerateSnapshot_Option1::RUN_MASTER;
                }
                else if (process->get_mode() == SnapshotProcess::Mode::SLAVE) {
                    state.CurrentCommand.Option1 =
                        (uint16_t)Command::GenerateSnapshot_Option1::RUN_SLAVE;
                }
                state.State = (uint8_t)eros::SnapshotState::INCOMPLETE;
                state.PercentComplete = 0.0;
                state.diag = eros_diagnostic::DiagnosticUtility::convert(diag);
                if (state.diag.Level > (uint8_t)Level::Type::NOTICE) {
                    logger->log_diagnostic(diag);
                }
                commandstate_pub.publish(state);
            }
        }
        ros::Duration(1.0).sleep();
    }
}
void SnapshotNode::cleanup() {
    process->request_statechange(Node::State::FINISHED);
    process->cleanup();
    delete process;
    base_cleanup();
}
void signalinterrupt_handler(int sig) {
    printf("Killing SnapshotNode with Signal: %d\n", sig);
    kill_node = true;
    exit(0);
}
int main(int argc, char **argv) {
    signal(SIGINT, signalinterrupt_handler);
    signal(SIGTERM, signalinterrupt_handler);
    ros::init(argc, argv, "snapshot_node");
    SnapshotNode *node = new SnapshotNode();
    bool status = node->start();
    if (status == false) {
        return EXIT_FAILURE;
    }
    std::thread thread(&SnapshotNode::thread_loop, node);
    std::thread thread2(&SnapshotNode::thread_snapshotcreation, node);
    while ((status == true) and (kill_node == false)) {
        status = node->update(node->get_process()->get_nodestate());
    }
    node->cleanup();
    thread.detach();
    thread2.detach();
    delete node;
    return 0;
}
