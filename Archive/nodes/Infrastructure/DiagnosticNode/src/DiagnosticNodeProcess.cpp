#include "DiagnosticNodeProcess.h"
using namespace eros;
using namespace eros_nodes::Infrastructure;
DiagnosticNodeProcess::DiagnosticNodeProcess() {
}
DiagnosticNodeProcess::~DiagnosticNodeProcess() {
}
eros_diagnostic::Diagnostic DiagnosticNodeProcess::finish_initialization() {
    eros_diagnostic::Diagnostic diag = get_root_diagnostic();

    for (uint8_t i = 1; i < (uint8_t)(eros_diagnostic::DiagnosticType::END_OF_LIST); ++i) {
        DiagnosticTypeAggregate aggregate;
        eros_diagnostic::Diagnostic diag("NO DEVICE",
                                         "NO NODE",
                                         System::MainSystem::ROVER,
                                         System::SubSystem::ROBOT_CONTROLLER,
                                         System::Component::DIAGNOSTIC,
                                         (eros_diagnostic::DiagnosticType)i,
                                         eros_diagnostic::Message::NOERROR,
                                         Level::Type::INFO,
                                         "No Diagnostics Received Yet.");
        aggregate.worst_diag = diag;
        aggregate.worst_diag_key = build_key(diag);
        diagnostic_aggregator.insert(
            std::pair<eros_diagnostic::DiagnosticType, DiagnosticTypeAggregate>(
                (eros_diagnostic::DiagnosticType)i, aggregate));
    }
    return diag;
}
void DiagnosticNodeProcess::reset() {
}
eros_diagnostic::Diagnostic DiagnosticNodeProcess::update(double t_dt, double t_ros_time) {
    eros_diagnostic::Diagnostic diag = base_update(t_dt, t_ros_time);
    ready_to_arm.ready_to_arm = true;
    ready_to_arm.diag = eros_diagnostic::DiagnosticUtility::convert(diag);
    return diag;
}
std::vector<eros_diagnostic::Diagnostic> DiagnosticNodeProcess::new_commandmsg(eros::command msg) {
    std::vector<eros_diagnostic::Diagnostic> diag_list = base_new_commandmsg(msg);
    if (diag_list.size() == 0) {
        // No currently supported commands.
    }
    else {
        for (auto diag : diag_list) {
            if (diag.level >= Level::Type::INFO) {
                diagnostic_manager.update_diagnostic(diag);
            }
        }
    }
    return diag_list;
}
std::vector<eros_diagnostic::Diagnostic> DiagnosticNodeProcess::check_programvariables() {
    std::vector<eros_diagnostic::Diagnostic> diag_list;
    logger->log_warn("No Program Variables Checked.");
    return diag_list;
}
eros_diagnostic::Diagnostic DiagnosticNodeProcess::get_worst_diagnostic(
    eros_diagnostic::DiagnosticType type) {
    eros_diagnostic::Diagnostic empty_diag("NO DEVICE",
                                           "NO NODE",
                                           System::MainSystem::ROVER,
                                           System::SubSystem::ROBOT_CONTROLLER,
                                           System::Component::DIAGNOSTIC,
                                           eros_diagnostic::DiagnosticType::UNKNOWN,
                                           eros_diagnostic::Message::END_OF_LIST,
                                           Level::Type::FATAL,
                                           "Unknown Diagnostic Type: " + (uint8_t)type);
    auto aggregate_it = diagnostic_aggregator.find(type);
    if (aggregate_it == diagnostic_aggregator.end()) {
        // No practical way to unit test
        // LCOV_EXCL_START
        return empty_diag;
        // LCOV_EXCL_STOP
    }
    else {
        if (aggregate_it->second.update_count == 0) {
            eros_diagnostic::Diagnostic diag("NO DEVICE",
                                             "NO NODE",
                                             System::MainSystem::ROVER,
                                             System::SubSystem::ROBOT_CONTROLLER,
                                             System::Component::DIAGNOSTIC,
                                             type,
                                             eros_diagnostic::Message::NODATA,
                                             Level::Type::INFO,
                                             "No Diagnostics Received Yet.");
            return diag;
        }
        else {
            return aggregate_it->second.worst_diag;
        }
    }
    return empty_diag;
}
eros_diagnostic::Diagnostic DiagnosticNodeProcess::update_topiclist(
    std::vector<std::string> &new_diagnostic_topics_to_subscribe) {
    eros_diagnostic::Diagnostic diag = get_root_diagnostic();
    std::vector<std::string> new_diagnostic_topics;
    for (std::size_t i = 0; i < new_diagnostic_topics_to_subscribe.size(); ++i) {
        bool add_me = true;
        for (std::size_t j = 0; j < diagnostic_topics_already_monitoring.size(); ++j) {
            if (diagnostic_topics_already_monitoring.at(j) ==
                new_diagnostic_topics_to_subscribe.at(i)) {
                add_me = false;
            }
        }
        if (add_me == true) {
            new_diagnostic_topics.push_back(new_diagnostic_topics_to_subscribe.at(i));
            diagnostic_topics_already_monitoring.push_back(
                new_diagnostic_topics_to_subscribe.at(i));
        }
    }
    new_diagnostic_topics_to_subscribe = new_diagnostic_topics;
    return diag;
}
bool DiagnosticNodeProcess::new_external_diagnostic(eros_diagnostic::Diagnostic diag) {
    auto aggregate_it = diagnostic_aggregator.find(diag.type);
    if (aggregate_it == diagnostic_aggregator.end()) {
        return false;
    }
    std::string key = build_key(diag);
    if ((diag.level > aggregate_it->second.worst_diag.level)) {
        aggregate_it->second.worst_diag = diag;
        aggregate_it->second.worst_diag_key = key;
    }
    else if (key == aggregate_it->second.worst_diag_key) {
        aggregate_it->second.worst_diag = diag;
    }

    auto diag_it = aggregate_it->second.diag_list.find(key);
    if (diag_it == aggregate_it->second.diag_list.end()) {
        aggregate_it->second.diag_list.insert(
            std::pair<std::string, eros_diagnostic::Diagnostic>(key, diag));
    }
    else {
        diag_it->second = diag;
    }
    update_diagnostic(eros_diagnostic::DiagnosticType::COMMUNICATIONS,
                      Level::Type::INFO,
                      eros_diagnostic::Message::NOERROR,
                      "Received External Diagnostic");
    aggregate_it->second.update_count++;
    return true;
}
std::string DiagnosticNodeProcess::pretty() {
    std::string str = "Node State: " + Node::NodeStateString(get_nodestate());
    str += "---Diagnostic Aggregates---\n";
    uint8_t i = 1;
    for (auto aggregate_it : diagnostic_aggregator) {
        str += "[" + std::to_string(i) + "/" + std::to_string(diagnostic_aggregator.size()) + "] " +
               eros_diagnostic::DiagnosticUtility::DiagnosticTypeString(aggregate_it.first) +
               " Update Count: " + std::to_string(aggregate_it.second.update_count) +
               " Worst Level: " + Level::LevelString(aggregate_it.second.worst_diag.level) + "\n";
        if (aggregate_it.second.update_count > 0) {
            if (aggregate_it.second.worst_diag.level > Level::Type::NOTICE) {
                str += "\tWorst: " +
                       eros_diagnostic::DiagnosticUtility::pretty(
                           "", aggregate_it.second.worst_diag, false) +
                       "\n";
            }
            uint16_t j = 1;
            for (auto diag_it : aggregate_it.second.diag_list) {
                str += "\t[" + std::to_string(j) + "/" +
                       std::to_string(aggregate_it.second.diag_list.size()) + "] " +
                       eros_diagnostic::DiagnosticUtility::pretty("", diag_it.second, false) + "\n";
                j++;
            }
        }

        i++;
    }
    return str;
}
std::string DiagnosticNodeProcess::build_key(eros_diagnostic::Diagnostic diag) {
    std::string key =
        diag.device_name + "_" + diag.node_name + "_" + std::to_string((uint8_t)diag.system) + "_" +
        std::to_string((uint8_t)diag.subsystem) + "_" + std::to_string((uint8_t)diag.component) +
        "_" + std::to_string((uint8_t)diag.type);
    return key;
}
