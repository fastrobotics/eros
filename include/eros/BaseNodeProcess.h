/*! \file BaseNodeProcess.h
 */

#pragma once
// BaseNodeProcess class
// C System Files
#include <stdlib.h>
#include <sys/time.h>

// C++ System Files
#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/trim.hpp>
#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>

#include "boost/date_time/posix_time/posix_time.hpp"
// ROS Base Functionality
#include "ros/ros.h"
#include "ros/time.h"
// ROS Messages
#include <eros/armed_state.h>
#include <eros/command.h>
#include <eros/command_state.h>
#include <eros/diagnostic.h>
#include <eros/file.h>
#include <eros/heartbeat.h>
#include <eros/loadfactor.h>
#include <eros/mode_state.h>
#include <eros/ready_to_arm.h>
#include <eros/resource.h>
#include <eros/uptime.h>
#include <std_msgs/Bool.h>

// ROS Services
#include <eros/srv_change_nodestate.h>
#include <eros/srv_device.h>
#include <eros/srv_filetransfer.h>
#include <eros/srv_firmware.h>
#include <eros/srv_get_diagnostics.h>
#include <eros/srv_logger_level.h>

// ROS Actions
#include <eros/system_commandAction.h>
// Project
#include <eros_diagnostic/Diagnostic.h>
#include <eros_diagnostic/DiagnosticManager.h>
#include <eros_diagnostic/DiagnosticUtility.h>
#include <eros_utility/ConvertUtility.h>
#include <eros_utility/CoreUtility.h>

#include <nlohmann/json.hpp>

#include "Logger.h"
#include "ResourceMonitor.h"
#include "eros_Definitions.h"

using json = nlohmann::json;
namespace eros {
/*! \class BaseNodeProcess BaseNodeProcess.h "BaseNodeProcess.h"
 *  \brief This is a BaseNodeProcess class.  All NodeProcess should be a derived class from this
 * BaseNodeProcess Class. */
class BaseNodeProcess
{
   public:
    BaseNodeProcess()
        : logger(nullptr),
          base_node_name(""),
          node_state(Node::State::START),
          diagnostic_manager(),
          unittest_running(false),
          run_time(0.0),
          system_time(0.0) {
        homeDir = getenv("HOME");
        ready_to_arm.ready_to_arm = false;
        ready_to_arm.diag.Description = "NOT INITIALIZED";
    }
    virtual ~BaseNodeProcess() {
    }
    // Constants

    // Enums

    // Structs

    // Initialization Functions

    /*! \brief Initializes Process.  Should be called right after instantiating variable. */
    void initialize(std::string t_base_node_name,
                    std::string t_node_name,
                    std::string t_hostname,
                    System::MainSystem t_system,
                    System::SubSystem t_subsystem,
                    System::Component t_component,
                    Logger* _logger) {
        base_node_name = t_base_node_name;
        node_state = Node::State::START;
        hostname = t_hostname;
        diagnostic_manager.initialize(eros_diagnostic::Diagnostic(
            t_hostname, t_node_name, t_system, t_subsystem, t_component));
        logger = _logger;
    }
    /*! \brief Derived Process Initialization */
    virtual eros_diagnostic::Diagnostic finish_initialization() = 0;
    bool enable_diagnostics(std::vector<eros_diagnostic::DiagnosticType> diagnostic_types) {
        return diagnostic_manager.enable_diagnostics(diagnostic_types);
    }

    /*! \brief Resets Process. Used for counters, timers, etc.*/
    virtual void reset() = 0;

    // Update Functions

    /*! \brief Update function must be implemented in Derived Process.  This is used for all state
     * machine logic, etc. */
    virtual eros_diagnostic::Diagnostic update(double t_dt, double t_ros_time) = 0;
    eros_diagnostic::Diagnostic update_diagnostic(eros_diagnostic::Diagnostic diag) {
        return diagnostic_manager.update_diagnostic(diag);
    }
    eros_diagnostic::Diagnostic update_diagnostic(eros_diagnostic::DiagnosticType diagnostic_type,
                                                  Level::Type level,
                                                  eros_diagnostic::Message message,
                                                  std::string description) {
        return diagnostic_manager.update_diagnostic(diagnostic_type, level, message, description);
    }
    eros_diagnostic::Diagnostic base_update(double t_dt, double t_system_time);

    // Attribute Functions
    Node::State get_nodestate() {
        return node_state;
    }
    double get_runtime() {
        return run_time;
    }
    double get_system_time() {
        return system_time;
    }
    double get_run_time() {
        return run_time;
    }
    eros::ready_to_arm get_ready_to_arm() {
        return ready_to_arm;
    }

    std::string get_hostname() {
        return hostname;
    }
    eros_diagnostic::Diagnostic get_root_diagnostic() {
        return diagnostic_manager.get_root_diagnostic();
    }
    std::vector<eros_diagnostic::Diagnostic> get_diagnostics() {
        return diagnostic_manager.get_diagnostics();
    }
    std::vector<eros_diagnostic::Diagnostic> get_latest_diagnostics() {
        return diagnostic_manager.get_latest_diagnostics();
    }

    Logger* get_logger() {
        return logger;
    }

    // Data Functions

    json read_configuration(std::string device_name,
                            bool include_self = true,
                            std::string file_path = "~/catkin_ws/src/config/DeviceList.json");

    static std::string sanitize_path(std::string path);

    static FileHelper::FileInfo read_file(std::string file_path);
    static FileHelper::FileInfo write_file(std::string full_path, char* bytes, uint64_t byte_count);
    std::vector<std::string> get_files_indir(std::string dir);

    // Message Functions

    //! Request a Node State Change
    /*!
      \param newstate The state to be changed to.
      \param override (Optional) Override State Change.  For use in special situations where there's
      no harm in over-riding state change.
      \return If the state change was successful (true) or not
      (false)
    */
    bool request_statechange(Node::State newstate, bool override = false);

    virtual std::vector<eros_diagnostic::Diagnostic> new_commandmsg(eros::command t_msg) = 0;

    // Support Functions
    /*! \brief Must be implemented in Derived Process.  Used for diagnostic testing LEVEL2 and for
     * basic checking of different variables, if they are initialized, etc. */
    virtual std::vector<eros_diagnostic::Diagnostic> check_programvariables() = 0;

    // Printing Functions
    virtual std::string pretty() = 0;

    // Destructors
    virtual void cleanup() = 0;
    void base_cleanup();

   protected:
    //! Base Update Function of all Node Process Classes.
    /*!
      \param t_dt The delta in the sample time.
      \param t_system_time The current system time.
      \return A Diagnostic reflecting the status of the function.
    */

    Logger* logger;
    std::string hostname;
    std::string base_node_name;
    Node::State node_state;
    eros_diagnostic::DiagnosticManager diagnostic_manager;

    bool unittest_running;
    eros::ready_to_arm ready_to_arm;

   private:
    double run_time, system_time;
    std::string homeDir;
};
}  // namespace eros