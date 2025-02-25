/*! \file MasterNode.h
 */
#ifndef MasterNode_H
#define MasterNode_H
// C System Files
// C++ System Files
// ROS Base Functionality
// ROS Messages
// Project
#include <eros/BaseNode.h>

#include "MasterNodeProcess.h"

/*!
 *  \addtogroup eros_nodes
 *  @{
 */

//! Namespace for Enhanced-ROS Nodes
namespace eros_nodes::Infrastructure {
/*! \class MasterNode MasterNode.h "MasterNode.h"
 *  \brief MasterNode is used to provide information about a host device.
 */
class MasterNode : public eros::BaseNode
{
   public:
    MasterNode();
    ~MasterNode();
    // Constants
    /*! \brief The base name of the Node.*/
    const std::string BASE_NODE_NAME = "master_node";

    /*! \brief The Major Release Version of the Node.*/
    const uint16_t MAJOR_RELEASE_VERSION = 0;

    /*! \brief The Minor Release Version of the Node.*/
    const uint16_t MINOR_RELEASE_VERSION = 2;

    /*! \brief The Build Number of the Node.*/
    const uint16_t BUILD_NUMBER = 2;

    /*! \brief A Description of the Firmware.*/
    const std::string FIRMWARE_DESCRIPTION = "Latest Rev: 24-Feb-2025";

    /*! \brief What System this Node falls under.*/
    const eros::System::MainSystem DIAGNOSTIC_SYSTEM = eros::System::MainSystem::ROVER;

    /*! \brief What Subsystem this Node falls under.*/
    const eros::System::SubSystem DIAGNOSTIC_SUBSYSTEM = eros::System::SubSystem::ENTIRE_SYSTEM;

    /*! \brief What Component this Node falls under.*/
    const eros::System::Component DIAGNOSTIC_COMPONENT = eros::System::Component::CONTROLLER;

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
    MasterNodeProcess* get_process() {
        return process;
    }

    // Utility Functions

    // Support Functions

    // Message Functions
    bool changenodestate_service(eros::srv_change_nodestate::Request& req,
                                 eros::srv_change_nodestate::Response& res);
    bool device_service(eros::srv_device::Request& req, eros::srv_device::Response& res);
    void system_commandAction_Callback(const eros::system_commandGoalConstPtr& goal);
    void command_Callback(const eros::command::ConstPtr& t_msg);

    // Destructors
    void cleanup();

    // Printing Functions
    std::string pretty() override;

   private:
    eros::eros_diagnostic::Diagnostic read_launchparameters();
    MasterNodeProcess* process;
    actionlib::SimpleActionServer<eros::system_commandAction> system_command_action_server;
    ros::ServiceServer device_server_srv;
    eros::ResourceMonitor* resource_available_monitor;
    ros::Publisher resource_available_pub;
    ros::Publisher loadfactor_pub;
};
}  // namespace eros_nodes::Infrastructure
#endif  // MasterNode_H
