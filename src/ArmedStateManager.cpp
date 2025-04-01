#include <eros/ArmedStateManager.h>

namespace eros {
ArmedStateManager::ArmedStateManager(std::string device_name,
                                     std::string node_name,
                                     System::MainSystem system,
                                     System::SubSystem subsystem,
                                     Logger* _logger,
                                     std::vector<std::string> ready_to_arm_list)
    : current_diagnostic(device_name,
                         node_name,
                         system,
                         subsystem,
                         System::Component::CONTROLLER,
                         eros_diagnostic::DiagnosticType::REMOTE_CONTROL,
                         eros_diagnostic::Message::INITIALIZING,
                         Level::Type::WARN,
                         "Initializing"),
      logger(_logger) {
    current_armed_state.state = ArmDisarm::Type::DISARMED_CANNOTARM;
    for (auto signal_name : ready_to_arm_list) {
        ReadyToArmSignal signal(signal_name);
        ready_to_arm_signals.insert({signal_name, signal});
    }
}
ArmedStateManager::~ArmedStateManager() {
}
eros_diagnostic::Diagnostic ArmedStateManager::update(double t_ros_time) {
    eros_diagnostic::Diagnostic diag = current_diagnostic;
    if (diag.message == eros_diagnostic::Message::INITIALIZING) {
        diag.message = eros_diagnostic::Message::NOERROR;
        diag.level = Level::Type::INFO;
        diag.description = "Updated";
    }
    current_diagnostic = diag;
    if (current_time < 0) {
        current_time = t_ros_time;
        return diag;
    }
    double prev_time = current_time;
    double delta_time = t_ros_time - prev_time;
    if (delta_time < 0) {
        diag.message = eros_diagnostic::Message::DIAGNOSTIC_FAILED;
        diag.level = Level::Type::ERROR;
        diag.description = "Went back in time!";
        current_diagnostic = diag;
        return diag;
    }
    current_time = t_ros_time;
    // Update all Timers for Ready to Arm Signals
    std::map<std::string, ReadyToArmSignal>::iterator it = ready_to_arm_signals.begin();
    bool all_ready_to_arm = true;
    bool any_ready_to_arm_timeout = false;
    for (; it != ready_to_arm_signals.end(); it++) {
        if (it->second.update_count == 0) {
            all_ready_to_arm = false;
        }
        else if (it->second.status == false) {
            all_ready_to_arm = false;
        }
        double delta_time = current_time - it->second.last_update_time;
        if (delta_time > ArmedStateManager::ARMED_SIGNAL_TIMEOUT_SEC) {
            any_ready_to_arm_timeout = true;
            it->second.signal_timeout = true;
        }
        else {
            it->second.signal_timeout = false;
        }
    }
    // State Machine for Armed State
    if ((all_ready_to_arm == false) || (any_ready_to_arm_timeout == true)) {
        auto prev_state = current_armed_state.state;
        current_armed_state.state = ArmDisarm::Type::DISARMED_CANNOTARM;
        if (prev_state != current_armed_state.state) {
            logger->log_warn("Armed State Changed from: " + ArmDisarm::ArmDisarmString(prev_state) +
                             " to: " + ArmDisarm::ArmDisarmString(current_armed_state.state));
        }
    }
    else if (current_armed_state.state == ArmDisarm::Type::DISARMED_CANNOTARM) {
        if ((all_ready_to_arm == true) && (any_ready_to_arm_timeout == false)) {
            auto prev_state = current_armed_state.state;
            current_armed_state.state = ArmDisarm::Type::DISARMED;
            if (prev_state != current_armed_state.state) {
                logger->log_notice(
                    "Armed State Changed from: " + ArmDisarm::ArmDisarmString(prev_state) +
                    " to: " + ArmDisarm::ArmDisarmString(current_armed_state.state));
            }
        }
    }
    else if (current_armed_state.state == ArmDisarm::Type::ARMING) {
        arming_timer += delta_time;
        if (arming_timer >= ArmedStateManager::ARMING_TIME_SEC) {
            auto prev_state = current_armed_state.state;
            current_armed_state.state = ArmDisarm::Type::ARMED;
            if (prev_state != current_armed_state.state) {
                logger->log_notice(
                    "Armed State Changed from: " + ArmDisarm::ArmDisarmString(prev_state) +
                    " to: " + ArmDisarm::ArmDisarmString(current_armed_state.state));
            }
        }
    }
    else if (current_armed_state.state == ArmDisarm::Type::DISARMING) {
        disarming_timer += delta_time;
        if (disarming_timer >= ArmedStateManager::DISARMING_TIME_SEC) {
            auto prev_state = current_armed_state.state;
            current_armed_state.state = ArmDisarm::Type::DISARMED;
            if (prev_state != current_armed_state.state) {
                logger->log_notice(
                    "Armed State Changed from: " + ArmDisarm::ArmDisarmString(prev_state) +
                    " to: " + ArmDisarm::ArmDisarmString(current_armed_state.state));
            }
        }
    }
    diag.message = eros_diagnostic::Message::NOERROR;
    diag.level = Level::Type::INFO;
    diag.description = "Updated";
    return diag;
}  // namespace eros
eros_diagnostic::Diagnostic ArmedStateManager::new_command(eros::command cmd) {
    auto diag = current_diagnostic;
    // Safe Armed State Change based on Commands
    if (cmd.Command == (uint16_t)Command::Type::ARM) {
        if (current_armed_state.state == ArmDisarm::Type::DISARMED) {
            auto prev_state = current_armed_state.state;
            current_armed_state.state = ArmDisarm::Type::ARMING;
            if (prev_state != current_armed_state.state) {
                logger->log_notice(
                    "Armed State Changed from: " + ArmDisarm::ArmDisarmString(prev_state) +
                    " to: " + ArmDisarm::ArmDisarmString(current_armed_state.state));
            }
            arming_timer = 0.0;
            diag.message = eros_diagnostic::Message::NOERROR;
            diag.level = Level::Type::INFO;
            diag.description = "Command Accepted";
            return diag;
        }
        else {
            diag.message = eros_diagnostic::Message::DROPPING_PACKETS;
            diag.level = Level::Type::WARN;
            if (current_armed_state.state == ArmDisarm::Type::DISARMED_CANNOTARM) {
                diag.description = "Command Arm Received but Current State is: " +
                                   ArmDisarm::ArmDisarmString(current_armed_state.state) +
                                   ".  Reasons: ";
                for (auto reason : get_cannotarm_reasons()) { diag.description += reason + ". "; }
            }
            else {
                diag.description = "Command Arm Received but Current State is: " +
                                   ArmDisarm::ArmDisarmString(current_armed_state.state);
            }

            return diag;
        }
    }
    else if (cmd.Command == (uint16_t)Command::Type::DISARM) {
        if (current_armed_state.state == ArmDisarm::Type::ARMED) {
            auto prev_state = current_armed_state.state;
            current_armed_state.state = ArmDisarm::Type::DISARMING;
            if (prev_state != current_armed_state.state) {
                logger->log_notice(
                    "Armed State Changed from: " + ArmDisarm::ArmDisarmString(prev_state) +
                    " to: " + ArmDisarm::ArmDisarmString(current_armed_state.state));
            }
            disarming_timer = 0.0;
            diag.message = eros_diagnostic::Message::NOERROR;
            diag.level = Level::Type::INFO;
            diag.description = "Command Accepted";
            return diag;
        }
        else {
            diag.message = eros_diagnostic::Message::DROPPING_PACKETS;
            diag.level = Level::Type::WARN;
            diag.description = "Command Disarm Received but Current State is: " +
                               ArmDisarm::ArmDisarmString(current_armed_state.state);
            return diag;
        }
    }
    else {
        diag.message = eros_diagnostic::Message::DROPPING_PACKETS;
        diag.level = Level::Type::WARN;
        diag.description = "Command: " + std::to_string((uint16_t)cmd.Command) + " Not Supported!";
        return diag;
    }
}

bool ArmedStateManager::new_ready_to_arm_msg(std::string signal, bool data) {
    std::map<std::string, ReadyToArmSignal>::iterator it;

    it = ready_to_arm_signals.find(signal);
    if (it != ready_to_arm_signals.end()) {
        it->second.status = data;
        it->second.last_update_time = current_time;
        it->second.update_count++;
        return true;
    }
    return false;
}
bool ArmedStateManager::reset() {
    arming_timer = 0.0;
    disarming_timer = 0.0;
    std::map<std::string, ReadyToArmSignal>::iterator it = ready_to_arm_signals.begin();
    for (; it != ready_to_arm_signals.end(); it++) {
        it->second.status = false;
        it->second.last_update_time = -1.0;
        it->second.update_count = 0;
        it->second.signal_timeout = true;
    }
    auto prev_state = current_armed_state.state;
    current_armed_state.state = ArmDisarm::Type::DISARMED_CANNOTARM;
    if (prev_state != current_armed_state.state) {
        logger->log_warn("Armed State Changed from: " + ArmDisarm::ArmDisarmString(prev_state) +
                         " to: " + ArmDisarm::ArmDisarmString(current_armed_state.state));
    }
    return true;
}
std::string ArmedStateManager::pretty() {
    std::string str = "-----Armed State Manager-----\n";
    str += "\tTime: " + std::to_string(current_time) + " (sec)\n";
    str += "\tReady to Arm Signals:\n";
    std::map<std::string, ReadyToArmSignal>::iterator it = ready_to_arm_signals.begin();

    for (; it != ready_to_arm_signals.end(); it++) {
        str += "\t\tSignal: " + it->second.signal_name +
               " Rx: " + std::to_string(it->second.update_count) +
               " Status: " + std::to_string(it->second.status) +
               " Last Update: " + std::to_string(it->second.last_update_time) + " (sec)" +
               " dT: " + std::to_string(current_time - it->second.last_update_time) + " (sec)" +
               " Timeout? " + std::to_string(it->second.signal_timeout) + "\n";
    }
    str += "\tArmed State: " + ArmDisarm::ArmDisarmString(current_armed_state.state) + "\n";
    str += "\tDiag: " + eros_diagnostic::DiagnosticUtility::pretty("", current_diagnostic, false) +
           "\n";
    return str;
}
std::vector<std::string> ArmedStateManager::get_cannotarm_reasons() {
    std::vector<std::string> reasons;
    eros_diagnostic::Diagnostic diag = current_diagnostic;
    if (diag.message == eros_diagnostic::Message::INITIALIZING) {
        std::string reason = "Still Initializing, waiting for an update.";
        reasons.push_back(reason);
    }
    else if (ready_to_arm_signals.size() == 0) {
        std::string reason = "No Ready to Arm Signals Defined!";
        reasons.push_back(reason);
    }
    else {
        std::map<std::string, ReadyToArmSignal>::iterator it = ready_to_arm_signals.begin();

        for (; it != ready_to_arm_signals.end(); it++) {
            if (it->second.update_count == 0) {
                std::string reason = "Signal: " + it->second.signal_name + " Never received Data!";
                reasons.push_back(reason);
            }
            else if (it->second.status == false) {
                std::string reason =
                    "Signal: " + it->second.signal_name + " received not Ready To Arm";
                reasons.push_back(reason);
            }
            else if (it->second.signal_timeout == true) {
                double delta_time = current_time - it->second.last_update_time;
                std::string reason = "Signal: " + it->second.signal_name +
                                     " Not received update in " + std::to_string(delta_time) +
                                     " (sec)!";
                reasons.push_back(reason);
            }
        }
    }

    return reasons;
}
}  // namespace eros