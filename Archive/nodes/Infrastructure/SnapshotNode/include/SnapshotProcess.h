/*! \file SnapshotProcess.h
 */
#ifndef SnapshotProcess_H
#define SnapshotProcess_H
#include <eros/BaseNodeProcess.h>
#include <eros_utility/CoreUtility.h>
#include <tinyxml.h>

#include <boost/thread.hpp>
#include <boost/thread/scoped_thread.hpp>
#include <fstream>
namespace eros_nodes::Infrastructure {
/*! \class SnapshotProcess SnapshotProcess.h "SnapshotProcess.h"
 *  \brief The process utility for the Snapshot Node. */
class SnapshotProcess : public eros::BaseNodeProcess
{
   public:
    SnapshotProcess()
        : nodeHandle(nullptr),
          robot_namespace(""),
          mode(Mode::UNKNOWN),
          architecture(eros::Architecture::Type::UNKNOWN),
          devicesnapshot_state(eros::SnapshotState::NOTRUNNING),
          systemsnapshot_state(eros::SnapshotState::NOTRUNNING),
          snapshot_progress_percent(0.0),
          holdcomplete_timer(0.0){};
    ~SnapshotProcess();
    // Constants
    const double HOLDCOMPLETE_TIME = 5.0;

    // Enums
    enum class Mode {
        UNKNOWN = 0,    /*!< Uninitialized value. */
        MASTER = 1,     /*!< The SnapshotNode is running as a Master. */
        SLAVE = 2,      /*!< The SnapshotNode is running as a Slave. */
        END_OF_LIST = 3 /*!< Last item of list. Used for Range Checks. */
    };
    // Structs
    /*! \struct ExecCommand
   \brief ExecCommand container.  Holds executable commands and where there output goes for a
   Snapshot.
   */
    struct ExecCommand {
        std::string command;
        std::string output_file;
    };
    /*! \struct SlaveDevice
    \brief SlaveDevice container, used for tracking Snapshot information and logic.
    */
    struct SlaveDevice {
        SlaveDevice(std::string _name)
            : name(_name), device_snapshot_generated(false), timer(0.0), devicesnapshot_path("") {
        }
        std::string name;
        bool device_snapshot_generated;
        bool device_snapshot_processed;
        double timer;
        std::string devicesnapshot_path;
    };
    /*! \struct SnapshotConfig
    \brief SnapshotConfig container, used for holding Snapshot Configuration parameters.
    */
    struct SnapshotConfig {
        std::string stage_directory;

        std::vector<SlaveDevice> snapshot_devices;
        std::vector<std::string> folders;
        std::vector<std::string> files;
        std::vector<ExecCommand> commands;
        std::vector<std::string> scripts;
        std::string systemsnapshot_path;
        std::string device_snapshot_path;
        std::string bagfile_directory;
        std::string active_device_snapshot_completepath;
    };

    // Initialization Functions
    eros::eros_diagnostic::Diagnostic finish_initialization();
    void reset();

    // Update Functions
    eros::eros_diagnostic::Diagnostic update(double t_dt, double t_ros_time);

    // Attribute Functions
    Mode get_mode() {
        return mode;
    }
    void set_mode(Mode v) {
        mode = v;
    }
    void set_architecture(eros::Architecture::Type v) {
        architecture = v;
    }
    SnapshotConfig get_snapshot_config() {
        return snapshot_config;
    }
    eros::SnapshotState get_devicesnapshot_state() {
        return devicesnapshot_state;
    }
    void set_devicesnapshot_state(eros::SnapshotState v) {
        devicesnapshot_state = v;
    }
    eros::SnapshotState get_systemsnapshot_state() {
        return systemsnapshot_state;
    }
    void set_systemsnapshot_state(eros::SnapshotState v) {
        systemsnapshot_state = v;
    }
    double get_snapshotprogress_percentage() {
        return snapshot_progress_percent;
    }
    bool set_nodeHandle(ros::NodeHandle* nh, std::string _robot_namespace) {
        nodeHandle = nh;
        robot_namespace = _robot_namespace;
        return true;
    }

    // Utility Functions

    // Support Functions
    std::vector<eros::eros_diagnostic::Diagnostic> createnew_snapshot();
    std::vector<eros::eros_diagnostic::Diagnostic> clear_snapshots();
    eros::eros_diagnostic::Diagnostic load_config(std::string file_path,
                                                  std::vector<std::string> override_devicenames);
    static Mode ModeType(std::string v) {
        if (v == "MASTER") {
            return Mode::MASTER;
        }
        else if (v == "SLAVE") {
            return Mode::SLAVE;
        }
        else {
            return Mode::UNKNOWN;
        }
    }
    static std::string ModeString(Mode v) {
        switch (v) {
            case Mode::UNKNOWN: return "UNKNOWN"; break;
            case Mode::MASTER: return "MASTER";
            case Mode::SLAVE: return "SLAVE";
            default: return ModeString(Mode::UNKNOWN); break;
        }
    }
    std::vector<eros::eros_diagnostic::Diagnostic> check_programvariables();

    // Message Functions
    std::vector<eros::eros_diagnostic::Diagnostic> new_commandmsg(eros::command t_msg);
    std::vector<eros::eros_diagnostic::Diagnostic> new_commandstatemsg(eros::command_state t_msg);

    // Destructors
    void cleanup() {
        base_cleanup();
        return;
    }

    // Printing Functions
    std::string pretty() override;

   private:
    int count_files_indirectory(std::string directory, std::string filter = "");
    ros::NodeHandle* nodeHandle;
    std::string robot_namespace;
    Mode mode;
    eros::Architecture::Type architecture;
    eros::SnapshotState devicesnapshot_state;
    eros::SnapshotState systemsnapshot_state;
    SnapshotConfig snapshot_config;
    double snapshot_progress_percent;
    double holdcomplete_timer;
};
}  // namespace eros_nodes::Infrastructure
#endif  // SnapshotProcess_H
