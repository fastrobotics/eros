/*! \file systemmonitor_node.h
 */
#pragma once
// C System Files
// C++ System Files
// ROS Base Functionality
// ROS Messages
// Project

#include <eros/BaseNode.h>

#include "SystemMonitorProcess.h"
namespace eros_nodes::SystemMonitor {
/*! \class SystemMonitorNode SystemMonitorNode.h "SystemMonitorNode.h"
 *  \brief The SystemMonitorNode is used to monitor the state of the System. */
class SystemMonitorNode : public eros::BaseNode
{
   public:
    SystemMonitorNode();
    ~SystemMonitorNode();

    // Constants
    /*! \brief The base name of the Node.*/
    const std::string BASE_NODE_NAME = "system_monitor";

    /*! \brief The Major Release Version of the Node.*/
    const uint16_t MAJOR_RELEASE_VERSION = 0;

    /*! \brief The Minor Release Version of the Node.*/
    const uint16_t MINOR_RELEASE_VERSION = 3;

    /*! \brief The Build Number of the Node.*/
    const uint16_t BUILD_NUMBER = 2;

    /*! \brief A Description of the Firmware.*/
    const std::string FIRMWARE_DESCRIPTION = "Latest Rev: 12-Feb-2025";

    /*! \brief What System this Node falls under.*/
    const eros::System::MainSystem DIAGNOSTIC_SYSTEM = eros::System::MainSystem::REMOTE_CONTROL;

    /*! \brief What Subsystem this Node falls under.*/
    const eros::System::SubSystem DIAGNOSTIC_SUBSYSTEM = eros::System::SubSystem::ROBOT_MONITOR;

    /*! \brief What Component this Node falls under.*/
    const eros::System::Component DIAGNOSTIC_COMPONENT = eros::System::Component::DIAGNOSTIC;

    // Enums

    // Structs

    // Initialization Functions
    bool start();
    bool init_screen();
    eros::eros_diagnostic::Diagnostic finish_initialization();

    // Update Functions
    bool run_loop1();
    bool run_loop2();
    bool run_loop3();
    bool run_001hz();
    bool run_01hz();
    bool run_01hz_noisy();
    bool run_1hz();
    bool run_10hz();
    void thread_loop();

    // Attribute Functions
    SystemMonitorProcess *get_process() {
        return process;
    }

    // Utility Functions

    // Support Functions
    eros::eros_diagnostic::Diagnostic rescan_nodes();

    // Message Functions
    void heartbeat_Callback(const eros::heartbeat::ConstPtr &msg);
    void resourceused_Callback(const eros::resource::ConstPtr &msg);
    void resourceavailable_Callback(const eros::resource::ConstPtr &msg);
    void loadfactor_Callback(const eros::loadfactor::ConstPtr &msg);
    bool changenodestate_service(eros::srv_change_nodestate::Request &req,
                                 eros::srv_change_nodestate::Response &res);
    void system_commandAction_Callback(const eros::system_commandGoalConstPtr &goal);
    void command_Callback(const eros::command::ConstPtr &t_msg);
    void commandState_Callback(const eros::command_state::ConstPtr &t_msg);

    // Destructors
    void cleanup();

    // Printing Functions
    std::string pretty() override;

   private:
    std::string extract_robotnamespace(std::string str);
    std::vector<ros::Subscriber> heartbeat_subs;
    std::vector<ros::Subscriber> resource_used_subs;
    std::vector<ros::Subscriber> resource_available_subs;
    std::vector<ros::Subscriber> loadfactor_subs;
    ros::Subscriber commandstate_sub;
    std::map<std::string, bool> filter_list;
    eros::eros_diagnostic::Diagnostic read_launchparameters();
    SystemMonitorProcess *process;
    actionlib::SimpleActionServer<eros::system_commandAction> system_command_action_server;
};
}  // namespace eros_nodes::SystemMonitor