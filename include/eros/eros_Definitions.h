/*! \file eros_Definitions.h
 */

#ifndef EROSDEFINITIONS_H
#define EROSDEFINITIONS_H

//! Define if ROS is Installed or Not.
#define ROS_INSTALLED
#include <string>

#include "ros/time.h"

/*!
 *  \addtogroup eros
 *  @{
 */

//! Enhanced-ROS Namespace
namespace eros {
/*! \struct ResourceInfo
   \brief ResourceInfo Information:
   Holds information about a Node's Resource Usage.
*/
struct ResourceInfo {
    /*@{*/
    ros::Time stamp;
    std::string process_name; /**< The name of the process. */
    uint64_t pid;             /**< The PID of the Process.  0 is Invalid. */
    double cpu_perc;  /**< CPU Usage of a Process in Percentage.  100% would indicate the process
                         is fully utilizing 1 CPU. */
    double ram_perc;  /**< RAM Usage of a Process in Percentage.  100% would indicate the process
                         is using 100% of the avaialble RAM. */
    double disk_perc; /**< Disk Usage of a Process in Percentage.  100% would indicate the
                         process is using 100% of the available disk space. */
    /*@}*/
};
/*! \struct ExecResult
\brief ExecResult Information:
Holds information about the result of an executed command
*/
struct ExecResult {
    ExecResult(bool anyError, std::string errorString, std::string result)
        : AnyError(anyError), ErrorString(errorString), Result(result) {
    }
    ExecResult() : AnyError(false), ErrorString(""), Result("") {
    }
    bool AnyError;
    std::string ErrorString;
    std::string Result;
};
/*! \struct Firmware
    \brief Firmware Information:
    Holds information about Firmware Version.
*/
struct Firmware {
    /*@{*/
    uint16_t MajorVersion;   /**< Incremented when a change IS NOT Backwards Compatible. */
    uint16_t MinorVersion;   /**< Incremented when a change IS Backwards Compatible. */
    uint16_t BuildNumber;    /**< Incremented during development.  */
    std::string Description; /**< User defined.  */
    /*@}*/
};
/*! \class FileHelper
    \brief FileHelper Information:
    Holds definiitions and and support functions for dealing with Files.
*/
class FileHelper
{
   public:
    enum class FileType {
        UNKNOWN = 0,    /*!< Uninitialized value. */
        ZIP = 1,        /*!< A Zip File */
        END_OF_LIST = 2 /*!< Last item of list. Used for Range Checks. */
    };

    enum class FileStatus {
        UNKNOWN = 0,    /*!< Uninitialized value. */
        FILE_OK = 1,    /*!< File Ok. */
        FILE_ERROR = 2, /*!< File Error. */
        END_OF_LIST = 3 /*!< Last item of list. Used for Range Checks. */
    };

    /*! \struct FileInfo
        \brief FileInfo Container
        Holds information about Files.
    */
    struct FileInfo {
        FileInfo()
            : fileType(FileType::UNKNOWN),
              fileStatus(FileStatus::UNKNOWN),
              file_name(""),
              folder(""),
              full_path(""),
              byte_size(0),
              data(nullptr) {};

        FileType fileType;
        FileStatus fileStatus;
        std::string file_name;
        std::string folder;
        std::string full_path;
        uint64_t byte_size;
        char* data;
    };
    static std::string pretty(FileInfo info) {
        std::string str;
        str = "Full Path: " + info.full_path + "\n";
        str += "File Name: " + info.file_name + "\n";
        str += "Folder: " + info.folder + "\n";
        return str;
    }

   private:
};
/*! \class System
    \brief System Information:
    Holds System and lower level definitions based on the Hierarchy:
   System <- System::SubSystem <- System::Component
*/
class System
{
   public:
    enum class MainSystem {
        UNKNOWN = 0,        /*!< Uninitialized value. */
        ROVER = 1,          /*!< Rover System. */
        SIMROVER = 2,       /*!< Simulated Rover System. */
        REMOTE_CONTROL = 3, /*!< Remote Control System. */
        END_OF_LIST = 4     /*!< Last item of list. Used for Range Checks. */
    };

    //! Convert System::MainSystem to human readable string
    /*!
      \param v System::MainSystem type
      \return The converted string.
    */
    static std::string MainSystemString(System::MainSystem v) {
        switch (v) {
            case System::MainSystem::UNKNOWN: return "UNKNOWN";
            case System::MainSystem::ROVER: return "ROVER";
            case System::MainSystem::SIMROVER: return "SIMROVER";
            case System::MainSystem::REMOTE_CONTROL: return "REMOTE_CONTROL";
            default: return MainSystemString(System::MainSystem::UNKNOWN);
        }
    }

    enum class SubSystem {
        UNKNOWN = 0,          /*!< Uninitialized value. */
        ENTIRE_SYSTEM = 1,    /*!< Applies to Entire System .*/
        ROBOT_CONTROLLER = 2, /*!< A Controller Subsystem. */
        ROBOT_MONITOR = 3,    /*!< A Monitor Subsystem */
        END_OF_LIST = 4       /*!< Last item of list. Used for Range Checks. */
    };

    //! Convert System::SubSystem to human readable string
    /*!
      \param v System::SubSystem type
      \return The converted string.
    */
    static std::string SubSystemString(System::SubSystem v) {
        switch (v) {
            case System::SubSystem::UNKNOWN: return "UNKNOWN";
            case System::SubSystem::ENTIRE_SYSTEM: return "ENTIRE_SYSTEM";
            case System::SubSystem::ROBOT_CONTROLLER: return "ROBOT_CONTROLLER";
            case System::SubSystem::ROBOT_MONITOR: return "ROBOT_MONITOR";
            default: return SubSystemString(System::SubSystem::UNKNOWN);
        }
    }

    enum class Component {
        UNKNOWN = 0,          /*!< Uninitialized value. */
        ENTIRE_SUBSYSTEM = 1, /*!< Applies to entire SubSystem. */
        CONTROLLER = 2,       /*!< A Controller Component. */
        DIAGNOSTIC = 3,       /*!< A Diagnostic Component. */
        NAVIGATION = 4,       /*!< A Navigation Component, such as Navigation Planning, Navigation
                                 Execution, etc. */
        MAPPING = 5,          /*!< A Component used for Mapping. */
        LEARNING = 6,         /*!< A Component used for Machine Learning. */
        TARGETING = 7,        /*!< A Component used for Target Acquisition. */
        TIMING = 8,           /*!< A Component used for Timing, Time Synchronization, etc. */
        VISION = 9, /*!< A Component used for Perception, Image Capture, Video Capture, etc. */
        GPIO = 10,  /*!< A Component used for reading/writion Inputs and Outputs. */
        COMMUNICATION = 11, /*!< A Component used for Communication. */
        DYNAMICS = 12,      /*!< A Component used for monitoring system dynamics. */
        POWER = 13,         /*!< A Component used to monitor and report power state. */
        POSE = 14,          /*!< A Component used to compute Pose. */
        END_OF_LIST = 15    /*!< Last item of list. Used for Range Checks. */
    };

    //! Convert System::Component to human readable string
    /*!
      \param v System::Component type
      \return The converted string.
    */
    static std::string ComponentString(System::Component v) {
        switch (v) {
            case System::Component::UNKNOWN: return "UNKNOWN";
            case System::Component::ENTIRE_SUBSYSTEM: return "ENTIRE_SUBSYSTEM";
            case System::Component::CONTROLLER: return "CONTROLLER";
            case System::Component::DIAGNOSTIC: return "DIAGNOSTIC";
            case System::Component::NAVIGATION: return "NAVIGATION";
            case System::Component::MAPPING: return "MAPPING";
            case System::Component::LEARNING: return "LEARNING";
            case System::Component::TARGETING: return "TARGETING";
            case System::Component::TIMING: return "TIMING";
            case System::Component::VISION: return "VISION";
            case System::Component::GPIO: return "GPIO";
            case System::Component::COMMUNICATION: return "COMMUNICATION";
            case System::Component::DYNAMICS: return "DYNAMICS";
            case System::Component::POWER: return "POWER";
            case System::Component::POSE: return "POSE";
            default: return ComponentString(System::Component::UNKNOWN);
        }
    }
};
/*! \class Level
    \brief Level for Information and Diagnostics:
    A numerically ordered level of information.  1 is considered the least, END_OF_LIST-1 is
   considered the highest.

*/
class Level
{
   public:
    enum class Type {
        UNKNOWN = 0, /*!< Uninitialized value. */
        DEBUG = 1,   /*!< This Level is solely for development/debugging only. */
        INFO = 2,    /*!< This Level is purely for informational use only. */
        NOTICE = 3,  /*!< This Level is a higher form of information and does not imply that
                     // anything is wrong. */
        WARN = 4,    /*!< This Level implies that a program is not running as expected, but may
                     // continue to operate in a diminished capacity. */
        ERROR = 5,   /*!< This Level implies that a program will not initialize or some other kind
                     // of crash. */
        FATAL = 6,   /*!<   This Level implies that a program has failed so bad it can cause
                        injury   to itself or others. */
        END_OF_LIST = 7 /*!< Last item of list. Used for Range Checks. */
    };

    //! Convert Level::Type to human readable string
    /*!
      \param v Level::Type type
      \return The converted string.
    */
    static std::string LevelString(Level::Type v) {
        switch (v) {
            case Level::Type::UNKNOWN: return "UNKNOWN";
            case Level::Type::DEBUG: return "DEBUG";
            case Level::Type::INFO: return "INFO";
            case Level::Type::NOTICE: return "NOTICE";
            case Level::Type::WARN: return "WARN";
            case Level::Type::ERROR: return "ERROR";
            case Level::Type::FATAL: return "FATAL";
            default: return LevelString(Level::Type::UNKNOWN);
        }
    }
    static Level::Type LevelType(std::string level) {
        if (level == "DEBUG") {
            return Level::Type::DEBUG;
        }
        if (level == "INFO") {
            return Level::Type::INFO;
        }
        if (level == "NOTICE") {
            return Level::Type::NOTICE;
        }
        if (level == "WARN") {
            return Level::Type::WARN;
        }
        if (level == "ERROR") {
            return Level::Type::ERROR;
        }
        if (level == "FATAL") {
            return Level::Type::FATAL;
        }
        return Level::Type::UNKNOWN;
    }
};
/*! \class ArmDisarm
    \brief ArmDisarm:
    Used to Arm and Disarm Robot.
*/
class ArmDisarm
{
   public:
    enum class Type {
        UNKNOWN = 0,            /*!< Uninitialized value. */
        ARMED = 1,              /*!< System is Armed. */
        DISARMED_CANNOTARM = 2, /*!< System is Disarmed, and Cannot Arm for some Reason. */
        DISARMED = 3,           /*!< System is Disarmed.  Can Arm if commanded to. */
        DISARMING = 4,          /*!< System is Disarming. */
        ARMING = 5,             /*!< System is Arming. */
        END_OF_LIST = 6         /*!< Last item of list. Used for Range Checks. */
    };
    /*! \struct State
        \brief ArmDisarm Container
        Holds information about ArmDisarm.
    */
    struct State {
        State() : state(Type::UNKNOWN) {
        }
        Type state;
    };

    //! Convert ArmDisarm::Type to human readable string
    /*!
      \param v ArmDisarm::Type type
      \return The converted string.
    */
    static std::string ArmDisarmString(ArmDisarm::Type v) {
        switch (v) {
            case ArmDisarm::Type::UNKNOWN: return "UNKNOWN";
            case ArmDisarm::Type::ARMED: return "ARMED";
            case ArmDisarm::Type::DISARMED_CANNOTARM: return "DISARMED_CANNOTARM";
            case ArmDisarm::Type::DISARMED: return "DISARMED";
            case ArmDisarm::Type::DISARMING: return "DISARMING";
            case ArmDisarm::Type::ARMING: return "ARMING";
            default: return ArmDisarmString(ArmDisarm::Type::UNKNOWN);
        }
    }
};
/*! \class Command
    \brief Command:
    A Command is a general purpose order from the highest level of softare on the robot.
*/
class Command
{
   public:
    enum class Type {
        UNKNOWN = 0,            /*!< Uninitialized value. */
        NONE = 1,               /*!< Don't perform a Command. */
        RUN_DIAGNOSTIC = 2,     /*!< Run a System Diagnostic. */
        ACQUIRE_TARGET = 3,     /*!< Search/Acquire a Target. */
        ARM = 4,                /*!< Arm the System. */
        DISARM = 5,             /*!< Disarm the System. */
        CONFIGURE = 6,          /*!< Perform a Configuration Command. */
        RUN = 7,                /*!< Perform regular system operation. */
        STOPMOVEMENT = 8,       /*!< Stop Moving (Safely). */
        DRIVE = 9,              /*!< Perform a manual Drive Command. */
        RESET = 10,             /*!< Reset the System. */
        SETLOGLEVEL = 11,       /*!< Change the Log Level. */
        TASKCONTROL = 12,       /*!< Change the Task Execution mode. */
        CALIBRATION = 13,       /*!< Perform a Calibration Command. */
        GENERATE_SNAPSHOT = 14, /*!< Generate a System Snapshot. */
        WAIT = 15,              /*!< Wait for further instructions. */
        END_OF_LIST = 16        /*!< Last item of list. Used for Range Checks. */
    };

    enum class GenerateSnapshot_Option1 {
        UNKNOWN = 0,
        RUN_MASTER = 1,
        RUN_SLAVE = 2,
        RESET_MASTER = 3,
        RESET_SLAVE = 4,
        CLEAR_SNAPSHOTS = 5,
        END_OF_LIST = 6
    };

    //! Convert System::Type to human readable string
    /*!
      \param v Command::Type type
      \return The converted string.
    */
    static std::string CommandString(Command::Type v) {
        switch (v) {
            case Command::Type::UNKNOWN: return "UNKNOWN";
            case Command::Type::NONE: return "NONE";
            case Command::Type::RUN_DIAGNOSTIC: return "RUN_DIAGNOSTIC";
            case Command::Type::ACQUIRE_TARGET: return "ACQUIRE_TARGET";
            case Command::Type::ARM: return "ARM";
            case Command::Type::DISARM: return "DISARM";
            case Command::Type::CONFIGURE: return "CONFIGURE";
            case Command::Type::RUN: return "RUN";
            case Command::Type::STOPMOVEMENT: return "STOPMOVEMENT";
            case Command::Type::DRIVE: return "DRIVE";
            case Command::Type::RESET: return "RESET";
            case Command::Type::SETLOGLEVEL: return "SETLOGLEVEL";
            case Command::Type::TASKCONTROL: return "TASKCONTROL";
            case Command::Type::CALIBRATION: return "CALIBRATION";
            case Command::Type::GENERATE_SNAPSHOT: return "GENERATE_SNAPSHOT";
            case Command::Type::WAIT: return "WAIT";
            default: return CommandString(Command::Type::UNKNOWN);
        }
    }
};
/*! \class Node
    \brief Node:
    A Node contains information about the process being ran.
*/
class Node
{
   public:
    enum class State {
        UNKNOWN = 0,      /*!< Uninitialized value. */
        START = 1,        /*!< Node has just started. */
        INITIALIZING = 2, /*!< Node is initializing. */
        INITIALIZED = 3,  /*!< Node has finished initializing. */
        RUNNING = 4,      /*!< Node is running. */
        PAUSED = 5,       /*!< Node is paused. */
        RESET = 6,        /*!< Node is reset. */
        FINISHED = 7,     /*!< Node has finished cleanly. */
        END_OF_LIST = 8   /*!< Last item of list. Used for Range Checks. */
    };

    //! Convert Node::State to human readable string
    /*!
      \param v Node::State type
      \return The converted string.
    */
    static std::string NodeStateString(Node::State v) {
        switch (v) {
            case Node::State::UNKNOWN: return "UNKNOWN";
            case Node::State::START: return "START";
            case Node::State::INITIALIZING: return "INITIALIZING";
            case Node::State::INITIALIZED: return "INITIALIZED";
            case Node::State::RUNNING: return "RUNNING";
            case Node::State::PAUSED: return "PAUSED";
            case Node::State::RESET: return "RESET";
            case Node::State::FINISHED: return "FINISHED";
            default: return NodeStateString(Node::State::UNKNOWN);
        }
    }

    static Node::State NodeState(std::string state) {
        if (state == "START") {
            return Node::State::START;
        }
        if (state == "INITIALIZING") {
            return Node::State::INITIALIZING;
        }
        if (state == "INITIALIZED") {
            return Node::State::INITIALIZED;
        }
        if (state == "RUNNING") {
            return Node::State::RUNNING;
        }
        if (state == "PAUSED") {
            return Node::State::PAUSED;
        }
        if (state == "RESET") {
            return Node::State::RESET;
        }
        if (state == "FINISHED") {
            return Node::State::FINISHED;
        }
        return Node::State::UNKNOWN;
    }
};
/*! \class Architecture
    \brief Hardware Architecture Container:
    Stores information and provides relevant helper functions related to Hardware Architecture.
*/
class Architecture
{
   public:
    enum class Type {
        UNKNOWN = 0,    /*!< Uninitialized value. */
        X86_64 = 1,     /*!< x86 64-bit compatible */
        ARMV7L = 2,     /*!< Raspberry Pi 2, 3 */
        AARCH64 = 3,    /*!< Arch Linux 64-bit */
        END_OF_LIST = 4 /*!< Last item of list. Used for Range Checks. */
    };

    //! Convert Architecture::Type to human readable string
    /*!
      \param v Architecture::Type type
      \return The converted string.
    */
    static std::string ArchitectureString(Architecture::Type v) {
        switch (v) {
            case Architecture::Type::UNKNOWN: return "UNKNOWN";
            case Architecture::Type::X86_64: return "X86_64";
            case Architecture::Type::ARMV7L: return "ARMV7L";
            case Architecture::Type::AARCH64: return "AARCH64";
            default: return ArchitectureString(Architecture::Type::UNKNOWN);
        }
    }

    static Architecture::Type ArchitectureType(std::string type) {
        if (type == "X86_64") {
            return Architecture::Type::X86_64;
        }
        else if (type == "ARMV7L") {
            return Architecture::Type::ARMV7L;
        }
        else if (type == "AARCH64") {
            return Architecture::Type::AARCH64;
        }
        return Architecture::Type::UNKNOWN;
    }
};
enum class SnapshotState {
    UNKNOWN = 0,    /*!< Uninitialized value. */
    NOTRUNNING = 1, /*!< Snapshot is not Running. */
    STARTED = 2,    /*!< Snapshot was Started. */
    RUNNING = 3,    /*!< Snapshot is Running. */
    READY = 4,      /*!< Snapshot is ready for retreival. */
    COMPLETE = 5,   /*!< Snapshot was Completed. */
    INCOMPLETE = 6, /*!< Snapshot was unable to Complete. */
    END_OF_LIST = 7 /*!< Last item of list. Used for Range Checks. */
};
static std::string SnapshotStateString(SnapshotState v) {
    switch (v) {
        case SnapshotState::UNKNOWN: return "UNKNOWN";
        case SnapshotState::NOTRUNNING: return "NOT RUNNING";
        case SnapshotState::STARTED: return "STARTED";
        case SnapshotState::RUNNING: return "RUNNING";
        case SnapshotState::READY: return "READY";
        case SnapshotState::COMPLETE: return "COMPLETE";
        case SnapshotState::INCOMPLETE: return "INCOMPLETE";
        default: return SnapshotStateString(SnapshotState::UNKNOWN);
    }
};
}  // namespace eros
#endif  // EROSDEFINITIONS_H