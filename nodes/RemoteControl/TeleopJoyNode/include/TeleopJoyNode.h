/*! \file TeleopJoyNode.h
 */
#ifndef TeleopJoyNode_H
#define TeleopJoyNode_H
// C System Files
// C++ System Files
// ROS Base Functionality
// ROS Messages
// Project
#include <eros/BaseNode.h>

#include "TeleopJoyNodeProcess.h"
namespace eros_nodes::Infrastructure {
/*! \class TeleopJoyNode TeleopJoyNode.h "TeleopJoyNode.h"
 *  \brief The TeleopJoyNode is used to control a system Safely. */
class TeleopJoyNode : public eros::BaseNode
{
   public:
    TeleopJoyNode();
    ~TeleopJoyNode();
    // Constants
    /*! \brief The base name of the Node.*/
    const std::string BASE_NODE_NAME = "teleop_joy_node";

    /*! \brief The Major Release Version of the Node.*/
    const uint16_t MAJOR_RELEASE_VERSION = 0;

    /*! \brief The Minor Release Version of the Node.*/
    const uint16_t MINOR_RELEASE_VERSION = 0;

    /*! \brief The Build Number of the Node.*/
    const uint16_t BUILD_NUMBER = 0;

    /*! \brief A Description of the Firmware.*/
    const std::string FIRMWARE_DESCRIPTION = "Latest Rev: 26-March-2025";

    /*! \brief What System this Node falls under.*/
    const eros::System::MainSystem DIAGNOSTIC_SYSTEM = eros::System::MainSystem::REMOTE_CONTROL;

    /*! \brief What Subsystem this Node falls under.*/
    const eros::System::SubSystem DIAGNOSTIC_SUBSYSTEM = eros::System::SubSystem::ENTIRE_SYSTEM;

    /*! \brief What Component this Node falls under.*/
    const eros::System::Component DIAGNOSTIC_COMPONENT = eros::System::Component::COMMUNICATION;

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
    TeleopJoyNodeProcess* get_process() {
        return process;
    }

    // Utility Functions

    // Support Functions

    // Message Functions
    bool changenodestate_service(eros::srv_change_nodestate::Request& req,
                                 eros::srv_change_nodestate::Response& res);
    void system_commandAction_Callback(const eros::system_commandGoalConstPtr& goal);
    void command_Callback(const eros::command::ConstPtr& t_msg);
    void joy_Callback(const sensor_msgs::Joy::ConstPtr& t_msg);
    void ReadyToArmCallback(const eros::ready_to_arm::ConstPtr& msg, const std::string& topic_name);

    // Destructors
    void cleanup();

    // Printing Functions
    std::string pretty() override;

   private:
    eros::eros_diagnostic::Diagnostic read_launchparameters();
    ros::Subscriber command_sub;
    TeleopJoyNodeProcess* process;
    actionlib::SimpleActionServer<eros::system_commandAction> system_command_action_server;
    ros::Publisher armedstate_pub;
    std::vector<ros::Subscriber> ready_to_arm_subs;
    ros::Subscriber joy_sub;
    ros::Publisher cmd_vel_pub;
};
}  // namespace eros_nodes::Infrastructure
#endif  // TeleopJoyNode_H
