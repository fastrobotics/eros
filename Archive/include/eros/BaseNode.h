/*! \file BaseNode.h
 */
#ifndef EROS_BASENODE_H
#define EROS_BASENODE_H
// Base class
// C System Files
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include <csignal>

// C++ System Files
#include <iostream>
#include <thread>

// ROS Base Functionality
#include <actionlib/server/simple_action_server.h>

#include "ros/ros.h"
#include "ros/time.h"

// ROS Messages
#include <eros/armed_state.h>
#include <eros/mode_state.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
// Project
#include <eros/BaseNodeProcess.h>
namespace eros {

/*! \class BaseNode BaseNode.h "BaseNode.h"
 *  \brief This is a BaseNode class.  All Nodes should be a derived class from this Base Class.*/
class BaseNode
{
   public:
    BaseNode()
        : diagnostic(),
          n(new ros::NodeHandle("~")),
          robot_namespace("/"),
          host_name(),
          firmware_version(),
          base_node_name(""),
          node_name(""),
          deviceInfo(),
          pub_ready_to_arm(false),
          no_launch_enabled(false),
          logger(nullptr),
          logger_initialized(false),
          ros_rate(-1.0),
          loop1_enabled(false),
          loop1_rate(-1.0),
          loop2_enabled(false),
          loop2_rate(-1.0),
          loop3_enabled(false),
          loop3_rate(-1.0),
          disable_device_client(false),
          verbosity_level("DEBUG"),
          require_pps_to_start(false),
          pps_received(false),
          rand_delay_sec(0.0),
          armedstate_sub_disabled(false),
          armedstate_sub_rxtime(0.0),
          modestate_sub_disabled(false) {
        ready_to_arm.ready_to_arm = false;
        ready_to_arm.diag.Description = "NOT INITIALIZED";
    }
    virtual ~BaseNode() {
    }
    // Constants

    // Enums

    // Structs
    /*! \struct DeviceInfo
    \brief DeviceInfo Logic
    Container and logic for DeviceInfo processing.
    */
    struct DeviceInfo {
        DeviceInfo() : received(false) {
        }
        bool received;
    };
    // Initialization Functions
    /*! \brief Set if no launch file should be used.  Will use default values only.
     */
    void set_no_launch_enabled(bool v) {
        no_launch_enabled = v;
    }
    /*! \brief Set if Node should NOT Subscribe to ArmedState message.
     */
    void disable_armedstate_sub() {
        armedstate_sub_disabled = true;
    }
    /*! \brief Set if Node should NOT Subscribe to ModeState message.
     */
    void disable_modestate_sub() {
        modestate_sub_disabled = true;
    }
    /*! \brief Set if Node SHOULD Publish ArmedState message.
     */
    void enable_ready_to_arm_pub(bool v) {
        pub_ready_to_arm = v;
    }
    /*! \brief Set Node Base Name.  This will be the same for every instance of the node, and is
     * independent on where the node is run. This value is equivelant to the "type" field in the
     * launch file.
     */
    void set_basenodename(std::string t_base_node_name);

    /*! \brief Initializes Node Root Diagnostic. */
    void initialize_diagnostic(System::MainSystem t_system,
                               System::SubSystem t_subsystem,
                               System::Component t_component);
    /*! \brief Initializes firmware based on Major, Minor, Build Number and Description. */
    void initialize_firmware(uint16_t t_major_version,
                             uint16_t t_minor_version,
                             uint16_t t_build_number,
                             std::string t_description);

    /*! \brief Set the name of the Node. */
    void set_nodename(std::string t_node_name) {
        node_name = t_node_name;
    }

    /*! \brief Start the Node. */
    virtual bool start() = 0;

    /*! \brief Pre-initialization of node.  This section will create the default pub/subs for the
     * node, along with the logger. */
    eros_diagnostic::Diagnostic preinitialize_basenode();

    /*! \brief Set Loop1 Rate in Hz. */
    void set_loop1_rate(double t_rate) {
        loop1_rate = t_rate;
        loop1_enabled = true;
    }
    /*! \brief Set Loop2 Rate in Hz. */
    void set_loop2_rate(double t_rate) {
        loop2_rate = t_rate;
        loop2_enabled = true;
    }

    /*! \brief Set Loop3 Rate in Hz. */
    void set_loop3_rate(double t_rate) {
        loop3_rate = t_rate;
        loop3_enabled = true;
    }

    /*! \brief Set the spin rate of the Node. This should be higher than the combined rates of
     * Loop1-Loop3 (if they are used.*/
    void set_ros_rate(double t_rate) {
        ros_rate = t_rate;
    }
    /*! \brief Reset the Base Node */
    void base_reset();

    // Update Functions
    /*! \brief Main Node update section, will call all derived Node Loop Functions, and publish base
     * pubs such as firmware, heartbeat, etc. */
    bool update(Node::State node_state);

    /*! \brief Run code in Loop1 Function.  Loop1 Must be Implemented in Derived Node.*/
    virtual bool run_loop1() = 0;
    /*! \brief Run code in Loop2 Function.  Loop2 Must be Implemented in Derived Node.*/
    virtual bool run_loop2() = 0;
    /*! \brief Run code in Loop3 Function.  Loop3 Must be Implemented in Derived Node.*/
    virtual bool run_loop3() = 0;
    /*! \brief Run code at .01 Hz.  Loop .01Hz Must be Implemented in Derived Node.*/
    virtual bool run_001hz() = 0;
    /*! \brief Run code at .1 Hz Function.  Loop .1Hz Must be Implemented in Derived Node.*/
    virtual bool run_01hz() = 0;
    /*! \brief Run code at .1 Hz Function with a small random delay to prevent multiple things
 running all at the same time. .1Hz Noisy Must be Implemented in Derived Node.*/
    virtual bool run_01hz_noisy() = 0;
    /*! \brief Run code at .1 Hz. Loop 1Hz Must be Implemented in Derived Node.*/
    virtual bool run_1hz() = 0;
    /*! \brief Run code at 10 Hz. Loop 10Hz Must be Implemented in Derived Node.*/
    virtual bool run_10hz() = 0;
    /*! \brief Thread Loop Must be Implemented in Derived Node.*/
    virtual void thread_loop() = 0;

    // Attribute Functions
    std::string get_basenodename() {
        return base_node_name;
    }
    std::string get_nodename() {
        return node_name;
    }
    // No practical way to unit test
    // LCOV_EXCL_START
    std::string read_robotnamespace() {
        std::string _robot_namespace;
        std::string param_robot_namespace = n->getUnresolvedNamespace() + "/robot_namespace";

        if (n->getParam(param_robot_namespace, _robot_namespace) == false) {
            _robot_namespace = "/";
        }

        _robot_namespace = validate_robotnamespace(_robot_namespace);
        return _robot_namespace;
    }
    // LCOV_EXCL_STOP
    void set_robotnamespace(std::string _robot_namespace) {
        robot_namespace = validate_robotnamespace(_robot_namespace);
    }
    std::string get_robotnamespace() {
        return robot_namespace;
    }
    // For some reason gcov is saying this isn't getting line coverage, but it assuredely is.
    // LCOV_EXCL_START
    static std::string validate_robotnamespace(std::string str);
    // LCOV_EXCL_STOP

    /*! \brief Get the current logger verbosity level. */
    std::string get_verbositylevel() {
        return verbosity_level;
    }
    static std::string get_hostname() {
        char name[1024];
        name[1023] = '\0';
        gethostname(name, 1023);
        return std::string(name);
    }
    boost::shared_ptr<ros::NodeHandle> get_nodehandle() {
        return n;
    }
    Logger *get_logger() {
        return logger;
    }

    // Utility Functions

    // Message Functions
    /*! \brief Update the node's ready to arm information.  This should be updated at least at 10Hz.
     */
    void update_ready_to_arm(eros::ready_to_arm v) {
        ready_to_arm = v;
    }
    /*! \brief Handles receiving the 1 PPS Msg. */
    void new_ppsmsg(const std_msgs::Bool::ConstPtr &t_msg);

    // Service Functions
    /*! \brief A firmware service, used to get a Node's current firmware. */
    bool firmware_service(eros::srv_firmware::Request &req, eros::srv_firmware::Response &res);

    /*! \brief A logger level service, used to change the Node's logger level. */
    bool loggerlevel_service(eros::srv_logger_level::Request &req,
                             eros::srv_logger_level::Response &res);

    /*! \brief A Diagnostics Service, used to get the node's current diagnostics. */
    bool diagnostics_service(eros::srv_get_diagnostics::Request &req,
                             eros::srv_get_diagnostics::Response &res);

    /*! \brief A Node State service, used to change the Node's state. */
    virtual bool changenodestate_service(eros::srv_change_nodestate::Request &req,
                                         eros::srv_change_nodestate::Response &res) = 0;

    /*! \brief Process an eros::command. */
    virtual void command_Callback(const eros::command::ConstPtr &t_msg) = 0;
    /*! \brief Process an eros::armed_state. */
    void armedstate_Callback(const eros::armed_state::ConstPtr &t_msg);
    /*! \brief Process an eros::mode_state. */
    void modestate_Callback(const eros::mode_state::ConstPtr &t_msg);

    virtual std::string pretty() = 0;

    // Destructors
    virtual void cleanup() = 0;
    void base_cleanup();

   protected:
    /*! \brief Get Base Launch parameters, which includes loop rates, verbosity, etc. */
    eros_diagnostic::Diagnostic read_baselaunchparameters();
    void update_diagnostics(std::vector<eros_diagnostic::Diagnostic> _diagnostics) {
        current_diagnostics = _diagnostics;
    }
    eros_diagnostic::Diagnostic diagnostic;

    boost::shared_ptr<ros::NodeHandle> n;
    std::string robot_namespace;
    std::string host_name;
    Firmware firmware_version;
    std::string base_node_name;
    std::string node_name;
    DeviceInfo deviceInfo;
    ros::Publisher state_pub;
    ros::Publisher heartbeat_pub;
    ros::Publisher diagnostic_pub;
    ros::Publisher resource_used_pub;

    bool pub_ready_to_arm;
    ros::Publisher readytoarm_pub;
    eros::ready_to_arm ready_to_arm;
    ros::Subscriber command_sub;

    eros::heartbeat heartbeat;
    ros::ServiceServer firmware_srv;
    ros::ServiceServer logger_level_srv;
    ros::ServiceServer diagnostics_srv;
    ros::ServiceServer nodestate_srv;
    bool no_launch_enabled;
    Logger *logger = nullptr;
    ResourceMonitor *resource_monitor = nullptr;
    bool logger_initialized;
    double ros_rate;

    ros::Time boot_time;
    ros::Time last_001hz_timer;
    ros::Time last_01hz_timer;
    ros::Time last_01hz_noisy_timer;
    ros::Time last_1hz_timer;
    ros::Time last_10hz_timer;
    bool loop1_enabled;
    ros::Time last_loop1_timer;
    double loop1_rate;

    bool loop2_enabled;
    ros::Time last_loop2_timer;
    double loop2_rate;

    bool loop3_enabled;
    double loop3_rate;
    ros::Time last_loop3_timer;

    bool disable_device_client;

    std::string verbosity_level;
    bool require_pps_to_start;
    bool pps_received;
    double rand_delay_sec;
    std::vector<eros_diagnostic::Diagnostic> current_diagnostics;

    bool armedstate_sub_disabled;
    ros::Subscriber armedstate_sub;
    double armedstate_sub_rxtime;
    eros::armed_state armed_state;
    bool modestate_sub_disabled;
    ros::Subscriber modestate_sub;
    eros::mode_state mode_state;

   private:
};
}  // namespace eros
#endif  // EROS_BASENODE_H
