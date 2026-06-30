#include <eros/BaseNode.h>
namespace eros {

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
bool BaseNode::preinitialize_basenode() {
    logger_initialized = false;
    node_name = ros::this_node::getName();
    heartbeat.HostName = host_name;
    heartbeat.BaseNodeName = base_node_name;
    heartbeat.NodeName = node_name;
    rand_delay_sec = (double)(rand() % 2000 - 1000) / 1000.0;

    std::string heartbeat_topic = node_name + "/heartbeat";
    heartbeat_pub = n->advertise<eros::heartbeat>(heartbeat_topic, 1);
    heartbeat.stamp = ros::Time::now();
    heartbeat.NodeState = (uint8_t)Node::State::INITIALIZING;
    heartbeat_pub.publish(heartbeat);

    std::string srv_firmware_topic = node_name + "/srv_firmware";
    firmware_srv = n->advertiseService(srv_firmware_topic, &BaseNode::firmware_service, this);

    std::string srv_loggerlevel_topic = node_name + "/srv_loggerlevel";
    logger_level_srv =
        n->advertiseService(srv_loggerlevel_topic, &BaseNode::loggerlevel_service, this);

    // LCOV_EXCL_STOP
    return true;
}
bool BaseNode::update(Node::State node_state) {
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
void BaseNode::new_ppsmsg(const std_msgs::Bool::ConstPtr& t_msg) {
    if (t_msg->data == true) {
        pps_received = true;
    }
}
// LCOV_EXCL_STOP
bool BaseNode::firmware_service(eros::srv_firmware::Request& req,
                                eros::srv_firmware::Response& res) {
    (void)req;  // No req information needed
    res.BaseNodeName = base_node_name;
    res.NodeName = node_name;
    res.MajorRelease = firmware_version.MajorVersion;
    res.MinorRelease = firmware_version.MinorVersion;
    res.BuildNumber = firmware_version.BuildNumber;
    res.Description = firmware_version.Description;
    return true;
}
bool BaseNode::loggerlevel_service(eros::srv_logger_level::Request& req,
                                   eros::srv_logger_level::Response& res) {
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
void BaseNode::base_cleanup() {
    for (int i = 0; i < 5; ++i) {
        heartbeat.NodeState = (uint8_t)Node::State::FINISHED;
        heartbeat.stamp = ros::Time::now();
        heartbeat_pub.publish(heartbeat);
    }
    delete resource_monitor;
    delete logger;
}
