/*! \file SafetyNodeProcess.h
 */
#pragma once
#include <eros/ArmedStateManager.h>
#include <eros/BaseNodeProcess.h>
#include <ros/ros.h>

namespace eros_nodes::Infrastructure {
/*! \class SafetyNodeProcess SafetyNodeProcess.h "SafetyNodeProcess.h"
 *  \brief The process utility for the Safety Node. */
class SafetyNodeProcess : public eros::BaseNodeProcess
{
   public:
    SafetyNodeProcess();
    ~SafetyNodeProcess();
    // Constants

    // Enums

    // Structs

    // Initialization Functions
    eros::eros_diagnostic::Diagnostic finish_initialization();
    void reset();
    bool set_ready_to_arm_signals(std::vector<std::string> signals);

    // Update Functions
    eros::eros_diagnostic::Diagnostic update(double t_dt, double t_ros_time);

    // Attribute Functions
    eros::armed_state get_armed_state() {
        return armed_state_manager->get_armed_state_msg();
    }
    std::vector<std::string> get_cannotarm_reasons() {
        return armed_state_manager->get_cannotarm_reasons();
    }

    // Utility Functions

    // Support Functions
    std::vector<eros::eros_diagnostic::Diagnostic> check_programvariables();

    // Message Functions
    std::vector<eros::eros_diagnostic::Diagnostic> new_commandmsg(eros::command msg);
    bool new_message_readytoarm(std::string name, eros::ready_to_arm ready_to_arm);
    bool new_message_readytoarm(std::string name, bool ready_to_arm);

    // Destructors
    void cleanup() {
        delete armed_state_manager;
        base_cleanup();
        return;
    }

    // Printing Functions
    std::string pretty() override;

   private:
    eros::ArmedStateManager* armed_state_manager;
};
}  // namespace eros_nodes::Infrastructure
