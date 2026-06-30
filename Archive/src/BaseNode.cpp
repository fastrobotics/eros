#include <eros/BaseNode.h>
using namespace eros;
std::string BaseNode::validate_robotnamespace(std::string str) {
    if (str == "") {
        str = "/";
    }
    if (str.at(0) != '/') {
        str.insert(str.begin(), '/');
    }
    if (str.at(str.length() - 1) != '/') {
        str.insert(str.length(), "/");
    }
    int max_count = str.size();
    bool search_duplicates = true;
    int counter = 0;
    while (search_duplicates == true) {
        bool duplicate_found = false;
        int index = 0;
        for (std::size_t i = 1; i < str.size(); ++i) {
            if ((str.at(i) == str.at(i - 1)) && (str.at(i) == '/')) {
                duplicate_found = true;
                index = i;
            }
        }
        if (duplicate_found == true) {
            str.erase(str.begin() + index);
        }
        else {  //(duplicate_found == false) {
            search_duplicates = false;
        }
        counter++;
        if (counter == max_count) {
            search_duplicates = false;
        }
    }

    return str;
}
void BaseNode::set_basenodename(std::string t_base_node_name) {
    base_node_name = t_base_node_name;
}
void BaseNode::initialize_firmware(uint16_t t_major_version,
                                   uint16_t t_minor_version,
                                   uint16_t t_build_number,
                                   std::string t_description) {
    firmware_version.MajorVersion = t_major_version;
    firmware_version.MinorVersion = t_minor_version;
    firmware_version.BuildNumber = t_build_number;
    firmware_version.Description = t_description;
}
void BaseNode::initialize_diagnostic(System::MainSystem t_system,
                                     System::SubSystem t_subsystem,
                                     System::Component t_component) {
    diagnostic.system = t_system;
    diagnostic.subsystem = t_subsystem;
    diagnostic.component = t_component;
}
void BaseNode::armedstate_Callback(const eros::armed_state::ConstPtr &t_msg) {
    armed_state = eros_utility::ConvertUtility::convert_fromptr(t_msg);
    armedstate_sub_rxtime = 0.0;
}
void BaseNode::modestate_Callback(const eros::mode_state::ConstPtr &t_msg) {
    mode_state = eros_utility::ConvertUtility::convert_fromptr(t_msg);
}

eros_diagnostic::Diagnostic BaseNode::preinitialize_basenode() {
    logger_initialized = false;
    require_pps_to_start = false;
    pps_received = false;
    node_name = ros::this_node::getName();
    boot_time = ros::Time::now();
    diagnostic.type = eros_diagnostic::DiagnosticType::SOFTWARE;
    diagnostic.level = Level::Type::INFO;
    diagnostic.message = eros_diagnostic::Message::INITIALIZING;
    diagnostic.description = "Node Initializing.";
    host_name = get_hostname();
    heartbeat.HostName = host_name;
    heartbeat.BaseNodeName = base_node_name;
    heartbeat.NodeName = node_name;
    rand_delay_sec = (double)(rand() % 2000 - 1000) / 1000.0;

    diagnostic = read_baselaunchparameters();
    // No Practical way to Unit Test
    // LCOV_EXCL_START
    if (diagnostic.level > Level::Type::WARN) {
        if (logger_initialized == true) {
            logger->log_diagnostic(diagnostic);
        }
        else {
            printf("[%s ERROR]: %s\n", node_name.c_str(), diagnostic.description.c_str());
        }
    }
    // LCOV_EXCL_STOP
    std::string heartbeat_topic = node_name + "/heartbeat";
    heartbeat_pub = n->advertise<eros::heartbeat>(heartbeat_topic, 1);
    heartbeat.stamp = ros::Time::now();
    heartbeat.NodeState = (uint8_t)Node::State::INITIALIZING;
    heartbeat_pub.publish(heartbeat);

    std::string diagnostic_topic = node_name + "/diagnostic";
    diagnostic_pub = n->advertise<eros::diagnostic>(diagnostic_topic, 20);

    std::string resource_used_topic = node_name + "/resource_used";
    resource_used_pub = n->advertise<eros::resource>(resource_used_topic, 2);

    std::string srv_firmware_topic = node_name + "/srv_firmware";
    firmware_srv = n->advertiseService(srv_firmware_topic, &BaseNode::firmware_service, this);

    std::string srv_loggerlevel_topic = node_name + "/srv_loggerlevel";
    logger_level_srv =
        n->advertiseService(srv_loggerlevel_topic, &BaseNode::loggerlevel_service, this);

    std::string srv_diagnostics_topic = node_name + "/srv_diagnostics";
    diagnostics_srv =
        n->advertiseService(srv_diagnostics_topic, &BaseNode::diagnostics_service, this);

    if (armedstate_sub_disabled == false) {
        armedstate_sub = n->subscribe<eros::armed_state>(
            get_robotnamespace() + "ArmedState", 10, &BaseNode::armedstate_Callback, this);
    }
    if (modestate_sub_disabled == false) {
        modestate_sub = n->subscribe<eros::mode_state>(
            get_robotnamespace() + "ModeState", 10, &BaseNode::modestate_Callback, this);
    }
    if (pub_ready_to_arm == true) {
        std::string readytoarm_topic = node_name + "/ready_to_arm";
        readytoarm_pub = n->advertise<eros::ready_to_arm>(readytoarm_topic, 1);
    }
    // No Practical way to Unit Test
    // LCOV_EXCL_START
    if (diagnostic.level > Level::Type::WARN) {
        if (logger_initialized == true) {
            logger->log_diagnostic(diagnostic);
        }
        else {
            printf("[%s] Could not complete pre-initialization. Exiting.\n", node_name.c_str());
        }
    }
    // LCOV_EXCL_STOP
    return diagnostic;
}
eros_diagnostic::Diagnostic BaseNode::read_baselaunchparameters() {
    eros_diagnostic::Diagnostic diag = diagnostic;
    loop1_enabled = false;
    loop2_enabled = false;
    loop3_enabled = false;
    if (no_launch_enabled == true) {
        logger = new Logger("NOTICE", node_name);
        logger_initialized = true;
    }
    else {
        std::string param_verbosity_level = node_name + "/verbosity_level";
        // No Practical way to Unit Test
        // LCOV_EXCL_START
        if (n->getParam(param_verbosity_level, verbosity_level) == false) {
            diag.type = eros_diagnostic::DiagnosticType::DATA_STORAGE;
            diag.level = Level::Type::ERROR;
            diag.message = eros_diagnostic::Message::INITIALIZING_ERROR;
            diag.description = "Missing Parameter: verbosity_level. Exiting.";
            diagnostic = diag;
            return diag;
        }
        // LCOV_EXCL_STOP
        else {
            logger = new Logger(verbosity_level, node_name);
            logger->log_notice("Initialized.");
            logger_initialized = true;
        }
        std::string param_robot_namespace = node_name + "/robot_namespace";
        // No Practical way to Unit Test
        // LCOV_EXCL_START
        if (n->getParam(param_robot_namespace, robot_namespace) == false) {
            robot_namespace = "/";
        }
        // LCOV_EXCL_STOP
        robot_namespace = validate_robotnamespace(robot_namespace);
    }

    resource_monitor = new ResourceMonitor(node_name, ResourceMonitor::Mode::PROCESS, diag, logger);
    diag = resource_monitor->init();
    // No Practical way to Unit Test
    // LCOV_EXCL_START
    if (diag.level >= Level::Type::ERROR) {
        logger->log_diagnostic(diag);
        return diag;
    }
    // LCOV_EXCL_STOP
    if (no_launch_enabled == true) {}
    else {
        std::string param_startup_delay = node_name + "/startup_delay";
        double startup_delay = 0.0;
        // No Practical way to Unit Test
        // LCOV_EXCL_START
        if (n->getParam(param_startup_delay, startup_delay) == false) {
            logger->log_notice("Missing Parameter: startup_delay.  Using Default: 0.0 sec.");
        }
        // LCOV_EXCL_STOP
        else {
            if (startup_delay > 0.1) {
                char tempstr[128];
                sprintf(tempstr, "Using Parameter: startup_delay = %4.2f sec.", startup_delay);
                logger->log_warn(std::string(tempstr));
            }
        }
        ros::Duration(startup_delay).sleep();
    }
    if (no_launch_enabled == true) {}
    else {
        std::string param_require_pps_to_start = node_name + "/require_pps_to_start";
        // No Practical way to Unit Test
        // LCOV_EXCL_START
        if (n->getParam(param_require_pps_to_start, require_pps_to_start) == false) {
            diag.type = eros_diagnostic::DiagnosticType::DATA_STORAGE;
            diag.level = Level::Type::ERROR;
            diag.message = eros_diagnostic::Message::INITIALIZING_ERROR;
            diag.description = "Missing Parameter: require_pps_to_start. Exiting.";
            diagnostic = diag;
            logger->log_diagnostic(diag);
        }
        // LCOV_EXCL_STOP
    }
    double max_rate = 0.0;
    if (no_launch_enabled == true) {
        loop1_enabled = true;
        loop1_rate = 1.0;
        loop2_enabled = true;
        loop2_rate = 0.2;
        loop3_enabled = false;
        max_rate = loop2_rate;
    }
    else {
        last_01hz_timer = ros::Time::now();
        last_01hz_noisy_timer = ros::Time::now();
        last_1hz_timer = ros::Time::now();
        last_10hz_timer = ros::Time::now();
        std::string param_loop1_rate = node_name + "/loop1_rate";
        // No Practical way to Unit Test
        // LCOV_EXCL_START
        if (n->getParam(param_loop1_rate, loop1_rate) == false) {
            logger->log_warn("Missing parameter: loop1_rate.  Not running loop1 code.");
            loop1_enabled = false;
        }
        // LCOV_EXCL_STOP
        else {
            last_loop1_timer = ros::Time::now();
            loop1_enabled = true;
            if (loop1_rate > max_rate) {
                max_rate = loop1_rate;
            }
        }

        std::string param_loop2_rate = node_name + "/loop2_rate";
        // No Practical way to Unit Test
        // LCOV_EXCL_START
        if (n->getParam(param_loop2_rate, loop2_rate) == false) {
            logger->log_warn("Missing parameter: loop2_rate.  Not running loop2 code.");
            loop2_enabled = false;
        }
        // LCOV_EXCL_STOP
        else {
            last_loop2_timer = ros::Time::now();
            loop2_enabled = true;
            if (loop2_rate > max_rate) {
                max_rate = loop2_rate;
            }
        }

        std::string param_loop3_rate = node_name + "/loop3_rate";
        // No Practical way to Unit Test
        // LCOV_EXCL_START
        if (n->getParam(param_loop3_rate, loop3_rate) == false) {
            logger->log_warn("Missing parameter: loop3_rate.  Not running loop3 code.");
            loop3_enabled = false;
        }
        // LCOV_EXCL_STOP
        else {
            last_loop3_timer = ros::Time::now();
            loop3_enabled = true;
            // No Practical way to Unit Test
            // LCOV_EXCL_START
            if (loop3_rate > max_rate) {
                max_rate = loop3_rate;
            }
            // LCOV_EXCL_STOP
        }
    }
    ros_rate = max_rate * 4.0;
    // No Practical way to Unit Test
    // LCOV_EXCL_START
    if (ros_rate > 100.0) {
        ros_rate = 100.0;
    }
    // LCOV_EXCL_STOP
    if (ros_rate <= 1.0) {
        ros_rate = 20.0;
    }
    char tempstr[512];
    sprintf(tempstr, "Running Node at Rate: %4.2f Hz.", ros_rate);
    logger->log_notice(std::string(tempstr));
    diagnostic = diag;
    return diag;
}

bool BaseNode::update(Node::State node_state) {
    bool ok_to_run = false;
    if (require_pps_to_start == false) {
        ok_to_run = true;
    }
    // No Practical way to Unit Test
    // LCOV_EXCL_START
    else if ((require_pps_to_start == true) and (pps_received == true)) {
        ok_to_run = true;
    }
    // LCOV_EXCL_STOP
    // No Practical way to Unit Test
    // LCOV_EXCL_START
    if (ok_to_run == false) {
        ros::Duration d(1.0);
        d.sleep();
        ros::spinOnce();
        logger->log_notice("Waiting on PPS To Start.");
        return true;
    }
    // LCOV_EXCL_STOP
    ros::Rate r(ros_rate);
    r.sleep();
    ros::spinOnce();
    double mtime = eros_utility::CoreUtility::measure_time_diff(ros::Time::now(), last_001hz_timer);

    if (mtime >= 100.0) {
        run_001hz();
        last_001hz_timer = ros::Time::now();
    }
    mtime = eros_utility::CoreUtility::measure_time_diff(ros::Time::now(), last_01hz_noisy_timer);
    if (mtime >= 10.0 + rand_delay_sec) {
        rand_delay_sec = (double)(rand() % 2000 - 1000) / 1000.0;
        run_01hz_noisy();
        resource_monitor->update(mtime);
        eros::resource resource_used =
            eros_utility::ConvertUtility::convert(resource_monitor->get_resourceinfo());
        resource_used_pub.publish(resource_used);
        std::vector<eros_diagnostic::Diagnostic> diag_list = current_diagnostics;
        for (std::size_t i = 0; i < diag_list.size(); ++i) {
            eros::diagnostic diag = eros_diagnostic::DiagnosticUtility::convert(diag_list.at(i));
            diagnostic_pub.publish(diag);
        }
        last_01hz_noisy_timer = ros::Time::now();
    }
    mtime = eros_utility::CoreUtility::measure_time_diff(ros::Time::now(), last_01hz_timer);
    if (mtime >= 10.0) {
        run_01hz();
        last_01hz_timer = ros::Time::now();
    }
    mtime = eros_utility::CoreUtility::measure_time_diff(ros::Time::now(), last_1hz_timer);
    if (mtime >= 1.0) {
        run_1hz();
        last_1hz_timer = ros::Time::now();
    }
    mtime = eros_utility::CoreUtility::measure_time_diff(ros::Time::now(), last_10hz_timer);
    if (mtime >= 0.1) {
        run_10hz();
        heartbeat.NodeState = (uint8_t)node_state;
        heartbeat.stamp = ros::Time::now();
        heartbeat_pub.publish(heartbeat);

        if (pub_ready_to_arm == true) {
            readytoarm_pub.publish(ready_to_arm);
        }
        if (armedstate_sub_disabled == false) {
            armedstate_sub_rxtime += 0.1;
            if (armedstate_sub_rxtime > 5.0) {
                armed_state.armed_state = (uint8_t)ArmDisarm::Type::UNKNOWN;
            }
        }
        last_10hz_timer = ros::Time::now();
    }

    if (loop1_enabled == true) {
        mtime = eros_utility::CoreUtility::measure_time_diff(ros::Time::now(), last_loop1_timer);
        if (mtime >= (1.0 / loop1_rate)) {
            run_loop1();
            last_loop1_timer = ros::Time::now();
        }
    }
    if (loop2_enabled == true) {
        mtime = eros_utility::CoreUtility::measure_time_diff(ros::Time::now(), last_loop2_timer);
        if (mtime >= (1.0 / loop2_rate)) {
            run_loop2();
            last_loop2_timer = ros::Time::now();
        }
    }
    if (loop3_enabled == true) {
        mtime = eros_utility::CoreUtility::measure_time_diff(ros::Time::now(), last_loop3_timer);
        if (mtime >= (1.0 / loop3_rate)) {
            run_loop3();
            last_loop3_timer = ros::Time::now();
        }
    }
    return ros::ok();
}
void BaseNode::base_reset() {
    resource_monitor->reset();
    eros::resource resource_used =
        eros_utility::ConvertUtility::convert(resource_monitor->get_resourceinfo());
    resource_used.stamp = ros::Time::now();
    resource_used_pub.publish(resource_used);
}
// No Practical way to Unit Test
// LCOV_EXCL_START
void BaseNode::new_ppsmsg(const std_msgs::Bool::ConstPtr &t_msg) {
    if (t_msg->data == true) {
        pps_received = true;
    }
}
// LCOV_EXCL_STOP
bool BaseNode::firmware_service(eros::srv_firmware::Request &req,
                                eros::srv_firmware::Response &res) {
    (void)req;  // No req information needed
    res.BaseNodeName = base_node_name;
    res.NodeName = node_name;
    res.MajorRelease = firmware_version.MajorVersion;
    res.MinorRelease = firmware_version.MinorVersion;
    res.BuildNumber = firmware_version.BuildNumber;
    res.Description = firmware_version.Description;
    return true;
}
bool BaseNode::loggerlevel_service(eros::srv_logger_level::Request &req,
                                   eros::srv_logger_level::Response &res) {
    Level::Type newLevel = Level::LevelType(req.LoggerLevel);
    if (newLevel == Level::Type::UNKNOWN) {
        res.Response = "Unsupported Logger Level: " + req.LoggerLevel;
        return false;
    }
    // No Practical way to Unit Test
    // LCOV_EXCL_START
    else if (logger == nullptr) {
        res.Response = "Logger is uninitialized.";
        return false;
    }
    // LCOV_EXCL_STOP
    else {
        logger->set_logverbosity(newLevel);
        res.Response = "Changed Logger Level to: " + req.LoggerLevel;
        return true;
    }
}
bool BaseNode::diagnostics_service(eros::srv_get_diagnostics::Request &req,
                                   eros::srv_get_diagnostics::Response &res) {
    eros_diagnostic::Diagnostic worst_diag;
    worst_diag.level = Level::Type::UNKNOWN;
    std::vector<eros_diagnostic::Diagnostic> diag_list = current_diagnostics;
    for (std::size_t i = 0; i < diag_list.size(); ++i) {
        if (diag_list.at(i).level > worst_diag.level) {
            worst_diag = diag_list.at(i);
        }
        eros::diagnostic diag = eros_diagnostic::DiagnosticUtility::convert(diag_list.at(i));
        bool add_me = false;
        if ((req.MinLevel == 0) || (req.DiagnosticType == 0)) {
            add_me = true;
        }
        // No Practical way to Unit Test
        // LCOV_EXCL_START
        else if ((diag.Level >= req.MinLevel)) {
            add_me = true;
        }
        else if (diag.DiagnosticType == req.DiagnosticType) {
            add_me = true;
        }
        // LCOV_EXCL_STOP
        if (add_me == true) {
            res.diag_list.push_back(diag);
        }
        logger->log_diagnostic(diag_list.at(i));
        diagnostic_pub.publish(diag);
    }
    res.worst_diag = eros_diagnostic::DiagnosticUtility::convert(worst_diag);

    return true;
}
void BaseNode::base_cleanup() {
    for (int i = 0; i < 5; ++i) {
        heartbeat.NodeState = (uint8_t)Node::State::FINISHED;
        heartbeat.stamp = ros::Time::now();
        heartbeat_pub.publish(heartbeat);
    }
    delete resource_monitor;
    delete logger;
}
