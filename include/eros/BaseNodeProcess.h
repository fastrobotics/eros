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

#include <nlohmann/json.hpp>

#include "Logger.h"
#include "eros_Definitions.h"

using json = nlohmann::json;
namespace eros {
/*! \class BaseNodeProcess BaseNodeProcess.h "BaseNodeProcess.h"
 *  \brief This is a BaseNodeProcess class.  All NodeProcess should be a derived class from this
 * BaseNodeProcess Class. */
class BaseNodeProcess
{
   public:
    BaseNodeProcess() : logger(nullptr), base_node_name(""), node_state(Node::State::START) {
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
        logger = _logger;
    }
    bool enable_commands(std::vector<Command::Type> commands) {
        supported_commands = commands;
        return true;
    }

    // Attribute Functions
    Node::State get_nodestate() {
        return node_state;
    }

    std::string get_hostname() {
        return hostname;
    }

    Logger* get_logger() {
        return logger;
    }
    std::vector<Command::Type> get_supported_commands() {
        return supported_commands;
    }

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

   private:
    Logger* logger;
    std::string hostname;
    std::string base_node_name;
    Node::State node_state;

   private:
    std::vector<Command::Type> supported_commands;
};
}  // namespace eros