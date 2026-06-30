/*! \file SafetyNode.h
 */
#ifndef SafetyNode_H
#define SafetyNode_H
// C System Files
// C++ System Files
// ROS Base Functionality
// ROS Messages
// Project
#include <eros/BaseNode.h>

#include "SafetyNodeProcess.h"
namespace eros_nodes::Infrastructure {
/*! \class SafetyNode SafetyNode.h "SafetyNode.h"
 *  \brief The SafetyNode is used to control a system Safely. */
class SafetyNode : public eros::BaseNode
{
   public:
    SafetyNode();
    ~SafetyNode();
    // Constants
    /*! \brief The base name of the Node.*/
    const std::string BASE_NODE_NAME = "safety_node";

    /*! \brief The Major Release Version of the Node.*/
    const uint16_t MAJOR_RELEASE_VERSION = 0;

    /*! \brief The Minor Release Version of the Node.*/
    const uint16_t MINOR_RELEASE_VERSION = 1;

    /*! \brief The Build Number of the Node.*/
    const uint16_t BUILD_NUMBER = 4;

    /*! \brief A Description of the Firmware.*/
    const std::string FIRMWARE_DESCRIPTION = "Latest Rev: 31-Mar-2025";

    /*! \brief What System this Node falls under.*/
    const eros::System::MainSystem DIAGNOSTIC_SYSTEM = eros::System::MainSystem::ROVER;

    /*! \brief What Subsystem this Node falls under.*/
    const eros::System::SubSystem DIAGNOSTIC_SUBSYSTEM = eros::System::SubSystem::ROBOT_MONITOR;

    /*! \brief What Component this Node falls under.*/
    const eros::System::Component DIAGNOSTIC_COMPONENT = eros::System::Component::DIAGNOSTIC;

    // Enums

    // Structs

    // Initialization Functions
    bool start();
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
    SafetyNodeProcess* get_process() {
        return process;
    }

    // Utility Functions

    // Support Functions

    // Message Functions
    bool changenodestate_service(eros::srv_change_nodestate::Request& req,
                                 eros::srv_change_nodestate::Response& res);
    void system_commandAction_Callback(const eros::system_commandGoalConstPtr& goal);
    void command_Callback(const eros::command::ConstPtr& t_msg);
    void ReadyToArmCallback(const eros::ready_to_arm::ConstPtr& msg, const std::string& topic_name);

    // Destructors
    void cleanup();

    // Printing Functions
    std::string pretty() override;

   private:
    eros::eros_diagnostic::Diagnostic read_launchparameters();
    ros::Subscriber command_sub;
    SafetyNodeProcess* process;
    actionlib::SimpleActionServer<eros::system_commandAction> system_command_action_server;
    ros::Publisher armedstate_pub;
    std::vector<ros::Subscriber> ready_to_arm_subs;
};
}  // namespace eros_nodes::Infrastructure
#endif  // SafetyNode_H
