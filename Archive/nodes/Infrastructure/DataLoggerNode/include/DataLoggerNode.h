/*! \file DataLoggerNode.h
 */
#ifndef DataLoggerNode_H
#define DataLoggerNode_H
// C System Files
// C++ System Files
// ROS Base Functionality
#include <rosbag/recorder.h>
// ROS Messages
#include <std_msgs/Empty.h>
// Project
#include <eros/BaseNode.h>

#include "DataLoggerProcess.h"
//! Enhanced-ROS Node Namespace
namespace eros_nodes::Infrastructure {
/*! \class DataLoggerNode DataLoggerNode.h "DataLoggerNode.h"
 *  \brief A Node that can be used to collect bag files.  Configured as either always logging to
 * disk, or snapshot mode where it will log to ram and write to disk when a snapshot trigger is
 * received.*/
class DataLoggerNode : public eros::BaseNode
{
   public:
    DataLoggerNode();
    ~DataLoggerNode();
    // Constants
    /*! \brief The base name of the Node.*/
    const std::string BASE_NODE_NAME = "datalogger_node";

    /*! \brief The Major Release Version of the Node.*/
    const uint16_t MAJOR_RELEASE_VERSION = 0;

    /*! \brief The Minor Release Version of the Node.*/
    const uint16_t MINOR_RELEASE_VERSION = 2;

    /*! \brief The Build Number of the Node.*/
    const uint16_t BUILD_NUMBER = 3;

    /*! \brief A Description of the Firmware.*/
    const std::string FIRMWARE_DESCRIPTION = "Latest Rev: 31-Mar-2025";

    /*! \brief What System this Node falls under.*/
    const eros::System::MainSystem DIAGNOSTIC_SYSTEM = eros::System::MainSystem::ROVER;

    /*! \brief What Subsystem this Node falls under.*/
    const eros::System::SubSystem DIAGNOSTIC_SUBSYSTEM = eros::System::SubSystem::ENTIRE_SYSTEM;

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
    DataLoggerProcess* get_process() {
        return process;
    }

    // Utility Functions

    // Support Functions
    void run_logger(DataLoggerNode* node);

    // Message Functions
    void snapshot_trigger_Callback(const std_msgs::Empty::ConstPtr& t_msg);
    bool changenodestate_service(eros::srv_change_nodestate::Request& req,
                                 eros::srv_change_nodestate::Response& res);
    void system_commandAction_Callback(const eros::system_commandGoalConstPtr& goal);
    void command_Callback(const eros::command::ConstPtr& t_msg);

    // Destructors
    void cleanup();

    // Printing Functions
    std::string pretty() override;

   private:
    bool create_snapshot_file();
    eros::eros_diagnostic::Diagnostic read_launchparameters();
    DataLoggerProcess* process;
    actionlib::SimpleActionServer<eros::system_commandAction> system_command_action_server;
    ros::Subscriber snapshot_trigger_sub;
};
}  // namespace eros_nodes::Infrastructure
#endif  // DataLoggerNode_H