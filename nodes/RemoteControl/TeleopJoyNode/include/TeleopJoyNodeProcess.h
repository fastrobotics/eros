/*! \file TeleopJoyNodeProcess.h
 */
#pragma once
#include <eros/BaseNodeProcess.h>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>

namespace eros_nodes::Infrastructure {
/*! \class TeleopJoyNodeProcess TeleopJoyNodeProcess.h "TeleopJoyNodeProcess.h"
 *  \brief The process utility for the Safety Node. */
class TeleopJoyNodeProcess : public eros::BaseNodeProcess
{
   public:
    TeleopJoyNodeProcess();
    ~TeleopJoyNodeProcess();
    // Constants

    // Enums

    // Structs
    struct CommandContainer {
        bool new_command{false};
        eros::command command;
    };

    // Initialization Functions
    eros::eros_diagnostic::Diagnostic finish_initialization();
    void reset();

    // Update Functions
    eros::eros_diagnostic::Diagnostic update(double t_dt, double t_ros_time);

    // Attribute Functions
    geometry_msgs::Twist get_cmd_twist() {
        return cmd_vel_perc;
    }
    CommandContainer get_current_command() {
        auto current_command = command;
        command.new_command = false;
        return current_command;
    }

    // Utility Functions

    // Support Functions
    std::vector<eros::eros_diagnostic::Diagnostic> check_programvariables();

    // Message Functions
    std::vector<eros::eros_diagnostic::Diagnostic> new_commandmsg(eros::command msg);
    std::vector<eros::eros_diagnostic::Diagnostic> new_joymsg(sensor_msgs::Joy msg);
    void update_armedstate(eros::ArmDisarm::State armed_state) {
        current_armed_state = armed_state;
    }

    // Destructors
    void cleanup() {
        base_cleanup();
        return;
    }

    // Printing Functions
    std::string pretty() override;

   private:
    geometry_msgs::Twist cmd_vel_perc;
    CommandContainer command;
    eros::ArmDisarm::State current_armed_state;
};
}  // namespace eros_nodes::Infrastructure
