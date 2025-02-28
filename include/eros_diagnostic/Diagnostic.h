/*! \file Diagnostic.h
 */
#pragma once
#include <eros/eros_Definitions.h>

#include <algorithm>
#include <string>
#include <vector>
//! Enhanced-ROS Diagnostic Namespace
namespace eros::eros_diagnostic {
enum class DiagnosticType {
    UNKNOWN = 0,          /*!< Uninitialized value. */
    UNKNOWN_TYPE = 1,     /*!< A Type that is not defined yet.*/
    SOFTWARE = 2,         /*!< Diagnostic related to Software. */
    COMMUNICATIONS = 3,   /*!< Diagnostic related to Communications. */
    SENSORS = 4,          /*!< Diagnostic related to Sensors. */
    ACTUATORS = 5,        /*!< Diagnostic related to Actuators. */
    DATA_STORAGE = 6,     /*!< Diagnostic related to Data Storage. */
    REMOTE_CONTROL = 7,   /*!< Diagnostic related to Remote Control. */
    TARGETING = 8,        /*!< Diagnostic related to Targeting. */
    POSE = 9,             /*!< Diagnostic related to Pose. */
    TIMING = 10,          /*!< *Diagnostic related to Timing. */
    SYSTEM_RESOURCE = 11, /*!< Diagnostic related to System Resource Usage. */
    END_OF_LIST = 12      /*!< Last item of list. Used for Range Checks. */
};

enum class Message {
    UNKNOWN = 0,              /*!< Uninitialized value. */
    NOERROR = 1,              /*!< No Error.*/
    NODATA = 2,               /*!< No Data.*/
    UNKNOWN_ERROR = 3,        /*!< Unknown Error Occurred/Error not defined. */
    INITIALIZING = 4,         /*!< Initializing. */
    INITIALIZING_ERROR = 5,   /*!< Error occured during initialization. */
    DROPPING_PACKETS = 6,     /*!< Missing/Dropping Messages. */
    MISSING_HEARTBEATS = 7,   /*!< Missing Heartbeat Messages. */
    DEVICE_NOT_AVAILABLE = 8, /*!< Device is not currently Available. */
    TEMPERATURE_HIGH = 9,     /*!< Temperature too High. */
    TEMPERATURE_LOW = 10,     /*!< Diagnostic Failed. */
    RESOURCE_LEAK = 11,       /*!< Resource Leakage occurred. */
    HIGH_RESOURCE_USAGE = 12, /*!< High Resource Usage. */
    DIAGNOSTIC_FAILED = 13,   /*! < Diagnostic Failed.*/
    END_OF_LIST = 14          /*!< Last item of list. Used for Range Checks. */
};

/*! \struct Diagnostic
\brief Contains the definition for the Diagnostic.

*/
struct Diagnostic {
    Diagnostic()
        : device_name(""),
          node_name(""),
          system(System::MainSystem::UNKNOWN),
          subsystem(System::SubSystem::UNKNOWN) {
    }
    Diagnostic(std::string _device_name,
               std::string _node_name,
               System::MainSystem _system,
               System::SubSystem _sub_system,
               System::Component _component,
               DiagnosticType _type,
               Message _message,
               Level::Type _level,
               std::string _description)
        : device_name(_device_name),
          node_name(_node_name),
          system(_system),
          subsystem(_sub_system),
          component(_component),
          type(_type),
          message(_message),
          level(_level),
          description(_description),
          update_count(1) {
    }
    /*!
      \param device_name The name of the Device.  Can be hostname or any name that makes sense for
      the device.
      \param node_name The name of the node running that has an instance of this class.
      \param system The System identifier, uses enum: System::MainSystem
      \param subsystem The SubSystem identifier, uses enum: System::SubSystem
      \param component The Component identifier, uses enum: System::Component
    */
    Diagnostic(std::string device_name,
               std::string node_name,
               System::MainSystem system,
               System::SubSystem subsystem,
               System::Component component)
        : device_name(device_name),
          node_name(node_name),
          system(system),
          subsystem(subsystem),
          component(component),
          type(DiagnosticType::UNKNOWN),
          message(Message::INITIALIZING),
          level(Level::Type::INFO),
          description(""),
          update_count(0) {
    }
    std::string device_name; /*!<  The name of the device that the node is currently running on. */

    std::string node_name;       /*!< The unique name of the node that has an instance of a
                                    Diagnostics Class. */
    System::MainSystem system;   /*!< The enum of a System the node represents.  */
    System::SubSystem subsystem; /*!< The enum of a Subsystem that that node represents. */
    System::Component component; /*!< The enum of a Component that that node represents. */
    DiagnosticType type; /*!< he enum of a Diagnostic Type that the node reports.  A Node can report
                            a diagnostic on multiple diagnostic types. */
    Message message;     /*!< A enum containing a summary of the Diagnostic. */
    Level::Type level;   /*!< Each Diagnostic should have a Level attached to it. */
    std::string description; /*!< A human readable string representing the diagnostic. */
    uint64_t update_count;   /*!<  */
};                           // namespace eros::eros_diagnostic

};  // namespace eros::eros_diagnostic