#include "SnapshotProcess.h"
using namespace eros;
using namespace eros_nodes::Infrastructure;
SnapshotProcess::~SnapshotProcess() {
}
eros_diagnostic::Diagnostic SnapshotProcess::finish_initialization() {
    eros_diagnostic::Diagnostic diag = diagnostic_manager.get_root_diagnostic();
    if (mode == Mode::UNKNOWN) {
        diag = diagnostic_manager.update_diagnostic(eros_diagnostic::DiagnosticType::DATA_STORAGE,
                                                    Level::Type::ERROR,
                                                    eros_diagnostic::Message::DEVICE_NOT_AVAILABLE,
                                                    "Mode not set!");
    }
    if (architecture == Architecture::Type::UNKNOWN) {
        diag = diagnostic_manager.update_diagnostic(eros_diagnostic::DiagnosticType::DATA_STORAGE,
                                                    Level::Type::ERROR,
                                                    eros_diagnostic::Message::DEVICE_NOT_AVAILABLE,
                                                    "Architecture not set!");
    }
    return diag;
}
void SnapshotProcess::reset() {
}
eros_diagnostic::Diagnostic SnapshotProcess::update(double t_dt, double t_ros_time) {
    eros_diagnostic::Diagnostic diag = base_update(t_dt, t_ros_time);
    if (devicesnapshot_state == SnapshotState::COMPLETE) {
        holdcomplete_timer += t_dt;
        if (holdcomplete_timer > HOLDCOMPLETE_TIME) {
            devicesnapshot_state = SnapshotState::NOTRUNNING;
            holdcomplete_timer = 0.0;
        }
    }
    ready_to_arm.ready_to_arm = true;
    ready_to_arm.diag = eros_diagnostic::DiagnosticUtility::convert(diag);
    return diag;
}
std::vector<eros_diagnostic::Diagnostic> SnapshotProcess::new_commandstatemsg(
    eros::command_state t_msg) {
    std::vector<eros_diagnostic::Diagnostic> diag_list;
    if (mode == Mode::MASTER) {
        if (t_msg.CurrentCommand.Command == (uint16_t)Command::Type::GENERATE_SNAPSHOT) {
            if (t_msg.CurrentCommand.Option1 ==
                (uint16_t)Command::GenerateSnapshot_Option1::RUN_SLAVE) {
                for (std::size_t i = 0; i < snapshot_config.snapshot_devices.size(); ++i) {
                    if (snapshot_config.snapshot_devices.at(i).name == t_msg.Name) {
                        if ((SnapshotState)t_msg.State == SnapshotState::COMPLETE) {
                            snapshot_config.snapshot_devices.at(i).device_snapshot_generated = true;
                            snapshot_config.snapshot_devices.at(i).devicesnapshot_path =
                                t_msg.CurrentCommand.CommandText;
                        }
                    }
                }
            }
        }
    }
    return diag_list;
}
std::vector<eros_diagnostic::Diagnostic> SnapshotProcess::new_commandmsg(eros::command t_msg) {
    eros_diagnostic::Diagnostic diag = get_root_diagnostic();
    std::vector<eros_diagnostic::Diagnostic> diag_list;
    if (t_msg.Command == (uint16_t)Command::Type::GENERATE_SNAPSHOT) {
        if (((mode == Mode::MASTER) &&
             (t_msg.Option1 == (uint16_t)Command::GenerateSnapshot_Option1::RUN_MASTER)) ||
            ((mode == Mode::SLAVE) &&
             (t_msg.Option1 == (uint16_t)Command::GenerateSnapshot_Option1::RUN_SLAVE))) {
            if ((systemsnapshot_state != SnapshotState::NOTRUNNING) ||
                (devicesnapshot_state != SnapshotState::NOTRUNNING)) {
                diag = update_diagnostic(eros_diagnostic::DiagnosticType::DATA_STORAGE,
                                         Level::Type::WARN,
                                         eros_diagnostic::Message::DROPPING_PACKETS,
                                         "Snapshot is still being generated.");
                diag_list.push_back(diag);
                return diag_list;
            }
            else {
                if (mode == Mode::MASTER) {
                    systemsnapshot_state = SnapshotState::STARTED;
                }
                devicesnapshot_state = SnapshotState::STARTED;
                diag = update_diagnostic(eros_diagnostic::DiagnosticType::DATA_STORAGE,
                                         Level::Type::INFO,
                                         eros_diagnostic::Message::NOERROR,
                                         "Snapshot Started.");
            }
        }
        else if ((t_msg.Option1 == (uint16_t)Command::GenerateSnapshot_Option1::CLEAR_SNAPSHOTS)) {
            diag_list = clear_snapshots();
        }
        else {
        }
    }
    else {
    }
    return diag_list;
}
std::vector<eros_diagnostic::Diagnostic> SnapshotProcess::check_programvariables() {
    std::vector<eros_diagnostic::Diagnostic> diag_list;
    return diag_list;
}
std::string SnapshotProcess::pretty() {
    std::string str = "Node State: " + Node::NodeStateString(get_nodestate());
    str += "--- Snapshot Config ---\n";
    str += " Mode: " + ModeString(mode) + "\n";
    if (mode == Mode::MASTER) {
        str += " System Snapshot State: " + SnapshotStateString(systemsnapshot_state) + "\n";
        if (snapshot_config.snapshot_devices.size() == 0) {
            str += " NO Snapshot Devices Defined.\n";
        }
        else {
            for (std::size_t i = 0; i < snapshot_config.snapshot_devices.size(); ++i) {
                str += " Device: " + snapshot_config.snapshot_devices.at(i).name + "\n";
            }
        }
    }
    str += " Device Snapshot State: " + SnapshotStateString(devicesnapshot_state) + "\n";
    str += " Stage Directory: " + snapshot_config.stage_directory + "\n";
    str += " Device Snapshot Path: " + snapshot_config.device_snapshot_path + "\n";
    if (snapshot_config.commands.size() == 0) {
        str += " No Commands Defined.\n";
    }
    else {
        str += " Commands:\n";
        for (std::size_t i = 0; i < snapshot_config.commands.size(); ++i) {
            str += "\t[" + std::to_string(i) + "/" +
                   std::to_string(snapshot_config.commands.size()) +
                   "] Cmd: " + snapshot_config.commands.at(i).command + "\n";
        }
    }
    if (snapshot_config.files.size() == 0) {
        str += " No Files Defined.\n";
    }
    else {
        str += " Files:\n";
        for (std::size_t i = 0; i < snapshot_config.files.size(); ++i) {
            str += "\t[" + std::to_string(i) + "/" + std::to_string(snapshot_config.files.size()) +
                   "] Cmd: " + snapshot_config.files.at(i) + "\n";
        }
    }
    if (snapshot_config.folders.size() == 0) {
        str += " No Folders Defined.\n";
    }
    else {
        str += " Folders:\n";
        for (std::size_t i = 0; i < snapshot_config.folders.size(); ++i) {
            str += "\t[" + std::to_string(i) + "/" +
                   std::to_string(snapshot_config.folders.size()) +
                   "] Cmd: " + snapshot_config.folders.at(i) + "\n";
        }
    }
    return str;
}
std::vector<eros_diagnostic::Diagnostic> SnapshotProcess::createnew_snapshot() {
    // logger->log_notice("starting");
    snapshot_progress_percent = 0.0;
    std::vector<eros_diagnostic::Diagnostic> diag_list;
    eros_diagnostic::Diagnostic diag = diagnostic_manager.get_root_diagnostic();
    if (mode == Mode::MASTER) {
        for (std::size_t i = 0; i < snapshot_config.snapshot_devices.size(); ++i) {
            snapshot_config.snapshot_devices.at(i).device_snapshot_generated = false;
        }
    }
    devicesnapshot_state = SnapshotState::RUNNING;
    // Clean up Stage Directory
    {
        try {
            std::string rm_cmd = "rm -r -f " + snapshot_config.stage_directory;
            eros_utility::CoreUtility::exec(rm_cmd.c_str(), true);
            std::string mkdir_cmd =
                "mkdir -p " + snapshot_config.stage_directory + "/DeviceSnapshot";
            eros_utility::CoreUtility::exec(mkdir_cmd.c_str(), true);
            mkdir_cmd = "mkdir -p " + snapshot_config.device_snapshot_path;
            eros_utility::CoreUtility::exec(mkdir_cmd.c_str(), true);
        }
        catch (const std::exception &e) {
            diag = update_diagnostic(eros_diagnostic::DiagnosticType::DATA_STORAGE,
                                     Level::Type::ERROR,
                                     eros_diagnostic::Message::DROPPING_PACKETS,
                                     "Command Exec failed with error: " + std::string(e.what()));
            diag_list.push_back(diag);
            return diag_list;
        }
    }
    snapshot_progress_percent = 5.0;
    // Run Snapshot "Commands"
    for (std::size_t i = 0; i < snapshot_config.commands.size(); ++i) {
        char tempstr[1024];
        sprintf(tempstr,
                "%s > %s/DeviceSnapshot/%s",
                snapshot_config.commands.at(i).command.c_str(),
                snapshot_config.stage_directory.c_str(),
                snapshot_config.commands.at(i).output_file.c_str());
        try {
            eros_utility::CoreUtility::exec(tempstr, true);
        }
        catch (const std::exception &e) {
            diag = update_diagnostic(eros_diagnostic::DiagnosticType::DATA_STORAGE,
                                     Level::Type::ERROR,
                                     eros_diagnostic::Message::DROPPING_PACKETS,
                                     "Command Exec failed with error: " + std::string(e.what()));
            diag_list.push_back(diag);
            return diag_list;
        }
    }
    snapshot_progress_percent = 25.0;
    // Snapshot File Copy
    for (std::size_t i = 0; i < snapshot_config.files.size(); ++i) {
        char tempstr[1024];
        sprintf(tempstr,
                "cp %s %s/DeviceSnapshot/",
                snapshot_config.files.at(i).c_str(),
                snapshot_config.stage_directory.c_str());
        try {
            eros_utility::CoreUtility::exec(tempstr, true);
        }
        catch (const std::exception &e) {
            diag = update_diagnostic(eros_diagnostic::DiagnosticType::DATA_STORAGE,
                                     Level::Type::ERROR,
                                     eros_diagnostic::Message::DROPPING_PACKETS,
                                     "File Copy Exec failed with error: " + std::string(e.what()));
            diag_list.push_back(diag);
            return diag_list;
        }
    }
    snapshot_progress_percent = 35.0;
    // Snapshot Folder Copy
    for (std::size_t i = 0; i < snapshot_config.folders.size(); ++i) {
        char tempstr[1024];
        sprintf(tempstr,
                "cp -r %s %s/DeviceSnapshot/",
                snapshot_config.folders.at(i).c_str(),
                snapshot_config.stage_directory.c_str());
        try {
            eros_utility::CoreUtility::exec(tempstr, true);
        }
        catch (const std::exception &e) {
            diag =
                update_diagnostic(eros_diagnostic::DiagnosticType::DATA_STORAGE,
                                  Level::Type::ERROR,
                                  eros_diagnostic::Message::DROPPING_PACKETS,
                                  "Folder Copy Exec failed with error: " + std::string(e.what()));
            diag_list.push_back(diag);
            return diag_list;
        }
    }
    snapshot_progress_percent = 45.0;

    time_t rawtime;
    struct tm *timeinfo;
    char buffer[80];

    time(&rawtime);
    timeinfo = localtime(&rawtime);

    strftime(buffer, sizeof(buffer), "Snapshot_%Y_%m_%d_%H_%M_%S", timeinfo);
    std::string time_str(buffer);
    std::string snapshot_name = get_hostname() + "_" + time_str;
    snapshot_config.active_device_snapshot_completepath = "";
    if (mode == Mode::SLAVE) {
        snapshot_progress_percent = 75.0;
    }
    // Zip up Device Snapshot
    if (1) {
        char tempstr[1024];
        sprintf(tempstr,
                "cd %s/DeviceSnapshot/ && zip -r %s/%s.zip .",
                snapshot_config.stage_directory.c_str(),
                snapshot_config.device_snapshot_path.c_str(),
                snapshot_name.c_str());
        snapshot_config.active_device_snapshot_completepath =
            snapshot_config.device_snapshot_path + snapshot_name + ".zip";
        eros_utility::CoreUtility::exec(tempstr, true);
    }
    if (mode == Mode::SLAVE) {
        snapshot_progress_percent = 95.0;
    }
    else if (mode == Mode::MASTER) {
        snapshot_progress_percent = 50.0;
    }
    // Make sure it's actually there
    {
        int file_found =
            count_files_indirectory(snapshot_config.device_snapshot_path, snapshot_name + ".zip");
        if (file_found != 1) {
            if (mode == Mode::SLAVE) {
                snapshot_progress_percent = 0.0;
            }
            diag = update_diagnostic(eros_diagnostic::DiagnosticType::DATA_STORAGE,
                                     Level::Type::WARN,
                                     eros_diagnostic::Message::DROPPING_PACKETS,
                                     "Device Snapshot not created at: " +
                                         snapshot_config.active_device_snapshot_completepath);
            diag_list.push_back(diag);
            return diag_list;
        }
        else {
            if (mode == Mode::SLAVE) {
                snapshot_progress_percent = 100.0;
            }
            devicesnapshot_state =
                SnapshotState::COMPLETE;  // Mark NOT RUNNING For now, may need to change when
                                          // System Snapshot gets implemented.
            diag = update_diagnostic(eros_diagnostic::DiagnosticType::DATA_STORAGE,
                                     Level::Type::INFO,
                                     eros_diagnostic::Message::NOERROR,
                                     "Device Snapshot Created at: " +
                                         snapshot_config.active_device_snapshot_completepath);
            diag_list.push_back(diag);
        }
    }
    if (mode == Mode::MASTER) {
        std::string systemsnap_name = "SystemSnap_" + time_str;
        double time_to_wait = 15.0 * (double)(snapshot_config.snapshot_devices.size());
        double perc_to_offset = 40.0 / (double)snapshot_config.snapshot_devices.size();
        double timer = 0.0;
        double dt = 0.1;
        bool all_complete = false;
        while (timer <= time_to_wait) {
            usleep(dt * 1000000.0);
            bool check = true;
            for (std::size_t i = 0; i < snapshot_config.snapshot_devices.size(); ++i) {
                check = check && snapshot_config.snapshot_devices.at(i).device_snapshot_generated;
                if (snapshot_config.snapshot_devices.at(i).device_snapshot_processed == false) {
                    snapshot_progress_percent += perc_to_offset;
                    snapshot_config.snapshot_devices.at(i).device_snapshot_processed = true;
                }
            }
            if (check == true) {
                all_complete = true;
                break;
            }
            timer += dt;
        }
        if (all_complete == true) {
            logger->log_warn("All Device Snapshot Processes Have Finished.");
        }
        else {
            for (std::size_t i = 0; i < snapshot_config.snapshot_devices.size(); ++i) {
                if (snapshot_config.snapshot_devices.at(i).device_snapshot_generated == false) {
                    logger->log_warn("Device: " + snapshot_config.snapshot_devices.at(i).name +
                                     " Has not completed in time.");
                    diag = update_diagnostic(eros_diagnostic::DiagnosticType::DATA_STORAGE,
                                             Level::Type::WARN,
                                             eros_diagnostic::Message::DROPPING_PACKETS,
                                             "Device Snapshot was not generated in time: " +
                                                 snapshot_config.snapshot_devices.at(i).name);
                    diag_list.push_back(diag);
                }
            }
        }
        // Move my own device snapshot to stage directory
        std::string mkdir_cmd = "mkdir -p " + snapshot_config.stage_directory + "/SystemSnapshot";
        eros_utility::CoreUtility::exec(mkdir_cmd.c_str(), true);
        std::string mv_cmd = "mv " + snapshot_config.active_device_snapshot_completepath + " " +
                             snapshot_config.stage_directory + "/SystemSnapshot";
        eros_utility::CoreUtility::exec(mv_cmd.c_str(), true);
        snapshot_progress_percent = 92.0;
        for (std::size_t i = 0; i < snapshot_config.snapshot_devices.size(); ++i) {
            // Request file
            ros::ServiceClient client = nodeHandle->serviceClient<eros::srv_filetransfer>(
                robot_namespace + snapshot_config.snapshot_devices.at(i).name +
                "/snapshot_node/srv_filetransfer");
            eros::srv_filetransfer req;
            req.request.path = snapshot_config.snapshot_devices.at(i).devicesnapshot_path;
            req.response.files.clear();
            if (client.call(req) == false) {
                logger->log_error("File Transfer Request from: " +
                                  snapshot_config.snapshot_devices.at(i).name + " Failed!");
            }
            else {
                for (std::size_t i = 0; i < req.response.files.size(); ++i) {
                    char arr[req.response.files.at(i).data_length];
                    std::copy(req.response.files.at(i).data.begin(),
                              req.response.files.at(i).data.end(),
                              arr);
                    FileHelper::FileInfo fileInfo = BaseNodeProcess::write_file(
                        snapshot_config.stage_directory + "/SystemSnapshot/" +
                            req.response.files.at(i).file_name,
                        arr,
                        req.response.files.at(i).data_length);
                    if (fileInfo.fileStatus != FileHelper::FileStatus::FILE_OK) {
                        logger->log_error(
                            "Not able to write file to path: " + snapshot_config.stage_directory +
                            "/SystemSnapshot/" + req.response.files.at(i).file_name);
                    }
                }
            }
        }
        if (count_files_indirectory(snapshot_config.stage_directory + "/SystemSnapshot/",
                                    "_Snapshot_") !=
            ((int)snapshot_config.snapshot_devices.size() + 1)) {
            diag = update_diagnostic(eros_diagnostic::DiagnosticType::DATA_STORAGE,
                                     Level::Type::WARN,
                                     eros_diagnostic::Message::DROPPING_PACKETS,
                                     "Device Snapshots are missing.");
            diag_list.push_back(diag);
            systemsnapshot_state = SnapshotState::INCOMPLETE;
        }
        snapshot_progress_percent = 95.0;

        // Copy Bag Files
        {
            std::string cp_cmd = "cp -r " + snapshot_config.bagfile_directory + "/* " +
                                 snapshot_config.stage_directory + "/SystemSnapshot/";
            eros_utility::CoreUtility::exec(cp_cmd.c_str(), true);
        }
        snapshot_progress_percent = 98.0;

        // Create System Snap Text File
        std::string result_file =
            snapshot_config.stage_directory + "/SystemSnapshot/SystemSnap.txt";

        logger->log_info("Creating System Snap Text File: " + result_file);
        std::ofstream result_file_fd(result_file.c_str(), std::ofstream::out);
        if (result_file_fd.is_open() == false) {
            diag = update_diagnostic(eros_diagnostic::DiagnosticType::DATA_STORAGE,
                                     Level::Type::WARN,
                                     eros_diagnostic::Message::DIAGNOSTIC_FAILED,
                                     "Unable to create System Snap Text File.");
            diag_list.push_back(diag);
            systemsnapshot_state = SnapshotState::INCOMPLETE;
        }
        else {
            result_file_fd << "System Snap Name: " << systemsnap_name << std::endl;
            std::vector<std::string> devices_complete;
            std::vector<std::string> devices_incomplete;
            for (std::size_t i = 0; i < snapshot_config.snapshot_devices.size(); ++i) {
                if (snapshot_config.snapshot_devices.at(i).device_snapshot_generated == true) {
                    devices_complete.push_back(snapshot_config.snapshot_devices.at(i).name);
                }
                else {
                    devices_incomplete.push_back(snapshot_config.snapshot_devices.at(i).name);
                }
            }
            if (devices_complete.size() == 0) {
                result_file_fd << "---" << std::endl
                               << "NO Device Snapshots Available!" << std::endl;
            }
            else {
                result_file_fd << "---" << std::endl << "Device Snapshots Completed:" << std::endl;
                int i = 0;
                for (auto dev : devices_complete) {
                    result_file_fd << "\t[" << std::to_string(i + 1) << "/"
                                   << std::to_string(devices_complete.size()) + "] " + dev
                                   << std::endl;
                    i++;
                }
            }
            if (devices_incomplete.size() == 0) {
                result_file_fd << "---" << std::endl
                               << "NO Device Snapshots were Incompleted." << std::endl;
            }
            else {
                result_file_fd << "---" << std::endl << "Device Snapshots Missing:" << std::endl;
                int i = 0;
                for (auto dev : devices_incomplete) {
                    result_file_fd << "\t[" << std::to_string(i + 1) << "/"
                                   << std::to_string(devices_incomplete.size()) + "] " + dev
                                   << std::endl;
                    i++;
                }
            }
            result_file_fd.close();
        }
        // Final Zip
        mkdir_cmd = "mkdir -p " + snapshot_config.systemsnapshot_path;
        eros_utility::CoreUtility::exec(mkdir_cmd.c_str(), true);
        char tempstr[1024];
        sprintf(tempstr,
                "cd %s/SystemSnapshot/ && zip -r %s/%s.zip .",
                snapshot_config.stage_directory.c_str(),
                snapshot_config.systemsnapshot_path.c_str(),
                systemsnap_name.c_str());
        logger->log_notice("Running: " + std::string(tempstr));
        eros_utility::CoreUtility::exec(tempstr, true);
        snapshot_progress_percent = 100.0;
        if (systemsnapshot_state != SnapshotState::INCOMPLETE) {
            systemsnapshot_state = SnapshotState::COMPLETE;
        }
    }
    return diag_list;
}
std::vector<eros_diagnostic::Diagnostic> SnapshotProcess::clear_snapshots() {
    eros_diagnostic::Diagnostic diag = diagnostic_manager.get_root_diagnostic();
    std::vector<eros_diagnostic::Diagnostic> diag_list;
    bool any_error = false;
    {
        std::string rm_cmd = "rm -r -f " + snapshot_config.device_snapshot_path + "/*";
        eros_utility::CoreUtility::exec(rm_cmd.c_str(), true);
        int file_count = count_files_indirectory(snapshot_config.device_snapshot_path + "/");
        if (file_count > 0) {
            any_error = true;
            diag =
                diagnostic_manager.update_diagnostic(eros_diagnostic::DiagnosticType::DATA_STORAGE,
                                                     Level::Type::WARN,
                                                     eros_diagnostic::Message::DIAGNOSTIC_FAILED,
                                                     "Command Failed: " + rm_cmd);
            diag_list.push_back(diag);
        }
    }
    {
        std::string rm_cmd = "rm -r -f " + snapshot_config.systemsnapshot_path + "/*";
        eros_utility::CoreUtility::exec(rm_cmd.c_str(), true);
        int file_count = count_files_indirectory(snapshot_config.systemsnapshot_path + "/");
        if (file_count > 0) {
            any_error = true;
            diag =
                diagnostic_manager.update_diagnostic(eros_diagnostic::DiagnosticType::DATA_STORAGE,
                                                     Level::Type::WARN,
                                                     eros_diagnostic::Message::DIAGNOSTIC_FAILED,
                                                     "Command Failed: " + rm_cmd);
            diag_list.push_back(diag);
        }
    }
    {
        std::string rm_cmd = "rm -r -f " + snapshot_config.stage_directory + "/DeviceSnapshot/*";
        eros_utility::CoreUtility::exec(rm_cmd.c_str(), true);
        int file_count =
            count_files_indirectory(snapshot_config.stage_directory + "/DeviceSnapshot/");
        if (file_count > 0) {
            any_error = true;
            diag =
                diagnostic_manager.update_diagnostic(eros_diagnostic::DiagnosticType::DATA_STORAGE,
                                                     Level::Type::WARN,
                                                     eros_diagnostic::Message::DIAGNOSTIC_FAILED,
                                                     "Command Failed: " + rm_cmd);
            diag_list.push_back(diag);
        }
    }
    {
        std::string rm_cmd = "rm -r -f " + snapshot_config.stage_directory + "/SystemSnapshot/*";
        eros_utility::CoreUtility::exec(rm_cmd.c_str(), true);
        int file_count =
            count_files_indirectory(snapshot_config.stage_directory + "/SystemSnapshot/");
        if (file_count > 0) {
            any_error = true;
            diag =
                diagnostic_manager.update_diagnostic(eros_diagnostic::DiagnosticType::DATA_STORAGE,
                                                     Level::Type::WARN,
                                                     eros_diagnostic::Message::DIAGNOSTIC_FAILED,
                                                     "Command Failed: " + rm_cmd);
            diag_list.push_back(diag);
        }
    }
    {
        std::string rm_cmd = "rm -r -f " + snapshot_config.bagfile_directory + "/*";
        eros_utility::CoreUtility::exec(rm_cmd.c_str(), true);
        int file_count = count_files_indirectory(snapshot_config.bagfile_directory + "/");
        if (file_count > 0) {
            any_error = true;
            diag =
                diagnostic_manager.update_diagnostic(eros_diagnostic::DiagnosticType::DATA_STORAGE,
                                                     Level::Type::WARN,
                                                     eros_diagnostic::Message::DIAGNOSTIC_FAILED,
                                                     "Command Failed: " + rm_cmd);
            diag_list.push_back(diag);
        }
    }
    if (any_error == false) {
        diag = diagnostic_manager.update_diagnostic(eros_diagnostic::DiagnosticType::DATA_STORAGE,
                                                    Level::Type::NOTICE,
                                                    eros_diagnostic::Message::NOERROR,
                                                    "Cleared Snapshot Directories.");

        diag_list.push_back(diag);
    }
    return diag_list;
}
eros_diagnostic::Diagnostic SnapshotProcess::load_config(
    std::string file_path, std::vector<std::string> override_devicenames) {
    logger->log_notice("Loading: " + file_path);
    file_path = sanitize_path(file_path);
    eros_diagnostic::Diagnostic diag = diagnostic_manager.get_root_diagnostic();
    TiXmlDocument doc(file_path);
    bool configfile_loaded = doc.LoadFile();
    if (configfile_loaded == false) {
        diag = update_diagnostic(eros_diagnostic::DiagnosticType::DATA_STORAGE,
                                 Level::Type::ERROR,
                                 eros_diagnostic::Message::INITIALIZING_ERROR,
                                 "Unable to load: " + file_path);
        return diag;
    }
    std::vector<std::string> missing_required_keys;
    std::vector<std::string> missing_optional_keys;
    TiXmlElement *l_pRootElement = doc.RootElement();
    bool found_architecture = false;
    if (nullptr != l_pRootElement) {
        TiXmlElement *l_pSnapshotConfig = l_pRootElement->FirstChildElement("SnapshotConfig");
        if (nullptr != l_pSnapshotConfig) {
            if (mode == Mode::MASTER) {
                if (override_devicenames.size() == 0) {
                    TiXmlElement *l_pSnapshotDevices =
                        l_pSnapshotConfig->FirstChildElement("SnapshotDevices");
                    if (nullptr != l_pSnapshotDevices) {
                        TiXmlElement *l_pSnapshotDevice =
                            l_pSnapshotDevices->FirstChildElement("Device");

                        while (l_pSnapshotDevice) {
                            std::string device_name = l_pSnapshotDevice->GetText();
                            if (device_name != get_hostname()) {
                                SlaveDevice newSlave(device_name);
                                snapshot_config.snapshot_devices.push_back(newSlave);
                            }
                            l_pSnapshotDevice = l_pSnapshotDevice->NextSiblingElement("Device");
                        }
                        /*
                        if (snapshot_config.snapshot_devices.size() == 0) {
                            missing_required_keys.push_back("SnapshotDevices/Device");
                        }
                        */
                    }
                    else {
                        missing_required_keys.push_back("SnapshotDevices");
                    }
                }
                else {
                    for (std::size_t i = 0; i < override_devicenames.size(); ++i) {
                        if (override_devicenames.at(i) != get_hostname()) {
                            SlaveDevice newSlave(override_devicenames.at(i));
                            snapshot_config.snapshot_devices.push_back(newSlave);
                        }
                    }
                }
            }
            TiXmlElement *l_pStageDirectory =
                l_pSnapshotConfig->FirstChildElement("StageDirectory");
            if (nullptr != l_pStageDirectory) {
                snapshot_config.stage_directory =
                    sanitize_path(std::string(l_pStageDirectory->GetText()));
            }
            else {
                missing_required_keys.push_back("StageDirectory");
            }
            TiXmlElement *l_pBagFileDirectory =
                l_pSnapshotConfig->FirstChildElement("BagFileDirectory");
            if (nullptr != l_pBagFileDirectory) {
                snapshot_config.bagfile_directory =
                    sanitize_path(std::string(l_pBagFileDirectory->GetText()));
            }
            else {
                missing_required_keys.push_back("BagFileDirectory");
            }
            TiXmlElement *l_pSystemSnapshotPath =
                l_pSnapshotConfig->FirstChildElement("SystemSnapshotPath");
            if (nullptr != l_pSystemSnapshotPath) {
                snapshot_config.systemsnapshot_path =
                    sanitize_path(std::string(l_pSystemSnapshotPath->GetText()));
            }
            else {
                if (mode == Mode::MASTER) {
                    missing_required_keys.push_back("SystemSnapshotPath");
                }
            }

            TiXmlElement *l_pArchitecture = l_pSnapshotConfig->FirstChildElement("Architecture");
            if (nullptr != l_pArchitecture) {
                while (l_pArchitecture) {
                    if (l_pArchitecture->Attribute("type") ==
                        Architecture::ArchitectureString(architecture)) {
                        found_architecture = true;
                        TiXmlElement *l_pDeviceSnapshotPath =
                            l_pArchitecture->FirstChildElement("DeviceSnapshotPath");
                        if (nullptr != l_pDeviceSnapshotPath) {
                            snapshot_config.device_snapshot_path =
                                sanitize_path(l_pDeviceSnapshotPath->GetText());
                        }
                        else {
                            missing_required_keys.push_back("DeviceSnapshotPath");
                        }
                        TiXmlElement *l_pFolder = l_pArchitecture->FirstChildElement("Folder");
                        if (nullptr != l_pFolder) {
                            while (l_pFolder) {
                                std::string folder = l_pFolder->GetText();
                                snapshot_config.folders.push_back(
                                    sanitize_path(std::string(l_pFolder->GetText())));
                                l_pFolder = l_pFolder->NextSiblingElement("Folder");
                            }
                        }
                        TiXmlElement *l_pFile = l_pArchitecture->FirstChildElement("File");
                        if (nullptr != l_pFile) {
                            while (l_pFile) {
                                snapshot_config.files.push_back(
                                    sanitize_path(std::string(l_pFile->GetText())));
                                l_pFile = l_pFile->NextSiblingElement("File");
                            }
                        }
                        TiXmlElement *l_pCommand = l_pArchitecture->FirstChildElement("Command");
                        if (nullptr != l_pCommand) {
                            while (l_pCommand) {
                                ExecCommand cmd;
                                cmd.command = l_pCommand->GetText();
                                cmd.output_file = sanitize_path(l_pCommand->Attribute("file"));
                                snapshot_config.commands.push_back(cmd);
                                l_pCommand = l_pCommand->NextSiblingElement("Command");
                            }
                        }
                    }
                    l_pArchitecture = l_pArchitecture->NextSiblingElement("Architecture");
                }
            }
        }
    }
    if (found_architecture == false) {
        missing_required_keys.push_back("Architecture");
    }
    if (missing_required_keys.size() == 0) {
        if (missing_optional_keys.size() == 0) {
            diag = update_diagnostic(eros_diagnostic::DiagnosticType::DATA_STORAGE,
                                     Level::Type::NOTICE,
                                     eros_diagnostic::Message::NOERROR,
                                     "Config File Loaded.");
        }
        else {
            std::string str = "";
            for (std::size_t i = 0; i < missing_optional_keys.size(); ++i) {
                if (i == 0) {
                    str = missing_optional_keys.at(i);
                }
                else {
                    str += "," + missing_optional_keys.at(i);
                }
            }
            diag = update_diagnostic(eros_diagnostic::DiagnosticType::DATA_STORAGE,
                                     Level::Type::WARN,
                                     eros_diagnostic::Message::NOERROR,
                                     "Missing Optional Keys: " + str);
        }
    }
    else {
        std::string str = "";
        for (std::size_t i = 0; i < missing_required_keys.size(); ++i) {
            if (i == 0) {
                str = missing_required_keys.at(i);
            }
            else {
                str += "," + missing_required_keys.at(i);
            }
        }
        diag = update_diagnostic(eros_diagnostic::DiagnosticType::DATA_STORAGE,
                                 Level::Type::ERROR,
                                 eros_diagnostic::Message::INITIALIZING_ERROR,
                                 "Missing Required Keys: " + str);
    }
    return diag;
}
int SnapshotProcess::count_files_indirectory(std::string directory, std::string filter) {
    try {
        char tempstr[1024];
        sprintf(tempstr, "ls %s*%s* 2>/dev/null | wc -l", directory.c_str(), filter.c_str());
        ExecResult res = eros_utility::CoreUtility::exec(tempstr, true);
        std::string return_v = res.Result;
        boost::trim_right(return_v);
        return std::atoi(return_v.c_str());
    }
    catch (const std::exception &e) {
        return 0;
    }
}
