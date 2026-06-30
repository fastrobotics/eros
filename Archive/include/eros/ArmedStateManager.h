/*! \file ArmedStateManager.h
 */
#pragma once
#include <eros/Logger.h>
#include <eros/armed_state.h>
#include <eros/command.h>
#include <eros_diagnostic/Diagnostic.h>
#include <eros_diagnostic/DiagnosticUtility.h>

namespace eros {
/*! \class ArmedStateManager
    \brief ArmedStateManager class is responsible for managing the robot's Armed State.
*/
class ArmedStateManager
{
   public:
    struct ReadyToArmSignal {
        ReadyToArmSignal(std::string signal_name)
            : signal_name(signal_name),
              status(false),
              last_update_time(-1.0),
              signal_timeout(false),
              update_count(0) {
        }
        std::string signal_name;
        bool status;
        double last_update_time;
        bool signal_timeout;
        uint64_t update_count;
    };
    ArmedStateManager(std::string device_name,
                      std::string node_name,
                      System::MainSystem system,
                      System::SubSystem subsystem,
                      Logger* logger,
                      std::vector<std::string> ready_to_arm_list);
    ~ArmedStateManager();
    // Constants
    static constexpr double ARMING_TIME_SEC = 5.0;
    static constexpr double DISARMING_TIME_SEC = 5.0;
    static constexpr double ARMED_SIGNAL_TIMEOUT_SEC = 2.5;
    // Enums

    // Structs

    // Initialization Functions
    bool reset();

    // Update Functions
    eros_diagnostic::Diagnostic update(double t_ros_time);

    // Attribute Functions
    eros_diagnostic::Diagnostic get_current_diagnostic() {
        return current_diagnostic;
    }
    ArmDisarm::State get_armed_state() {
        return current_armed_state;
    }
    armed_state get_armed_state_msg() {
        eros::armed_state msg;
        msg.armed_state = (uint8_t)current_armed_state.state;
        return msg;
    }
    std::vector<std::string> get_cannotarm_reasons();

    // Utility Functions

    // Support Functions

    // Message Functions
    eros_diagnostic::Diagnostic new_command(eros::command cmd);
    bool new_ready_to_arm_msg(std::string signal, bool data);

    // Destructors
    void cleanup() {
        return;
    }

    // Printing Functions
    std::string pretty();

   private:
    eros_diagnostic::Diagnostic current_diagnostic;
    Logger* logger;
    ArmDisarm::State current_armed_state;
    std::map<std::string, ReadyToArmSignal> ready_to_arm_signals;
    double current_time{-1.0};
    double arming_timer{-1.0};
    double disarming_timer{-1.0};
};
}  // namespace eros