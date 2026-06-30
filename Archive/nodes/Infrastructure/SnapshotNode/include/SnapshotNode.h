/*! \file SnapshotNode.h
 */
#ifndef SnapshotNode_H
#define SnapshotNode_H
// C System Files
// C++ System Files
// ROS Base Functionality
// ROS Messages
// Project
#include <eros/BaseNode.h>

#include "SnapshotProcess.h"
namespace eros_nodes::Infrastructure {
/*! \class SnapshotNode SnapshotNode.h "SnapshotNode.h"
 *  \brief The SnapshotNode is used to collect diagnostic information about a device for future
 * analysis. */
class SnapshotNode : public eros::BaseNode
{
   public:
    SnapshotNode();
    ~SnapshotNode();

    // Constants
    /*! \brief The base name of the Node.*/
    const std::string BASE_NODE_NAME = "snapshot_node";

    /*! \brief The Major Release Version of the Node.*/
    const uint16_t MAJOR_RELEASE_VERSION = 0;

    /*! \brief The Minor Release Version of the Node.*/
    const uint16_t MINOR_RELEASE_VERSION = 4;

    /*! \brief The Build Number of the Node.*/
    const uint16_t BUILD_NUMBER = 2;

    /*! \brief A Description of the Firmware.*/
    const std::string FIRMWARE_DESCRIPTION = "Latest Rev: 31-May-2025";

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
    void thread_snapshotcreation();

    // Attribute Functions
    SnapshotProcess* get_process() {
        return process;
    }

    // Utility Functions

    // Support Functions

    // Message Functions
    bool changenodestate_service(eros::srv_change_nodestate::Request& req,
                                 eros::srv_change_nodestate::Response& res);
    void system_commandAction_Callback(const eros::system_commandGoalConstPtr& goal);
    void command_Callback(const eros::command::ConstPtr& t_msg);
    void commandState_Callback(const eros::command_state::ConstPtr& t_msg);
    bool filetransfer_service(eros::srv_filetransfer::Request& req,
                              eros::srv_filetransfer::Response& res);

    // Destructors
    void cleanup();

    // Printing Functions
    std::string pretty() override;

   private:
    boost::shared_ptr<ros::NodeHandle> test_sp_handle;
    // ros::NodeHandle test_handle;
    eros::eros_diagnostic::Diagnostic read_launchparameters();
    SnapshotProcess* process;
    actionlib::SimpleActionServer<eros::system_commandAction> system_command_action_server;
    ros::ServiceServer filetransfer_srv;
    ros::Publisher commandstate_pub;
    ros::Subscriber commandstate_sub;
    ros::Publisher command_pub;
    ros::Publisher bagfile_snapshottrigger_pub;
};
}  // namespace eros_nodes::Infrastructure
#endif  // SnapshotNode_H
