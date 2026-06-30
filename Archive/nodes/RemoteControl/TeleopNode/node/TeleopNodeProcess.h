/*! \file TeleopNodeProcess.h
 */
#pragma once
#include <curses.h>
#include <eros/BaseNodeProcess.h>
// Windows
#include <eros_window/DiagnosticsWindow/DiagnosticsWindow.h>
#include <eros_window/HeaderWindow/HeaderWindow.h>
#include <eros_window/IWindow.h>
#include <eros_window/MessageWindow/MessageWindow.h>

#include "InstructionWindow/InstructionWindow.h"
#include "MainWindow/MainWindow.h"
namespace eros_nodes::RemoteControl {
/*! \class TeleopNodeProcess TeleopNodeProcess.h "TeleopNodeProcess.h"
 *  \brief
    The process utility for the DataLogger
 */
class TeleopNodeProcess : public eros::BaseNodeProcess
{
   public:
    TeleopNodeProcess()
        : kill_me(false),
          nodeHandle(nullptr),
          robot_namespace("/"),
          mainwindow_width(0),
          mainwindow_height(0) {
    }
    ~TeleopNodeProcess();
    // Constants
    /*! \brief The minimum width in pixels of the Main Window.*/
    const uint16_t MINWINDOW_WIDTH = 140;
    /*! \brief The minimum height in pixels of the Main Window.*/
    const uint16_t MINWINDOW_HEIGHT = 240;

    // Enums

    // Structs

    // Initialization Functions
    eros::eros_diagnostic::Diagnostic finish_initialization();
    void reset();
    bool initialize_windows();

    // Update Functions
    eros::eros_diagnostic::Diagnostic update(double t_dt, double t_ros_time);
    bool set_mainwindow(uint16_t t_mainwindow_width, uint16_t t_mainwindow_height) {
        mainwindow_width = t_mainwindow_width;
        mainwindow_height = t_mainwindow_height;
        if (mainwindow_width < MINWINDOW_WIDTH) {
            return false;
        }
        return true;
    }

    // Attribute Functions
    bool set_nodeHandle(ros::NodeHandle* nh, std::string _robot_namespace) {
        nodeHandle = nh;
        robot_namespace = _robot_namespace;
        std::string systemcommand_topic = robot_namespace + "SystemCommand";
        command_pub = nodeHandle->advertise<eros::command>(systemcommand_topic, 1);

        return true;
    }
    bool get_killme() {
        return kill_me;
    }
    void update_armedstate(eros::ArmDisarm::State armed_state);

    // Utility Functions

    // Support Functions
    std::vector<eros::eros_diagnostic::Diagnostic> check_programvariables();

    // Message Functions
    std::vector<eros::eros_diagnostic::Diagnostic> new_commandmsg(eros::command msg);
    eros::eros_diagnostic::Diagnostic new_commandstate(const eros::command_state::ConstPtr& t_msg);

    // Destructors
    void cleanup() {
        base_cleanup();
        endwin();
        return;
    }

    // Printing Functions
    std::string pretty() override;

   private:
    bool kill_me{false};
    ros::NodeHandle* nodeHandle;
    ros::Publisher command_pub;
    std::string robot_namespace;
    uint16_t mainwindow_width;
    uint16_t mainwindow_height;

    std::vector<eros_window::IWindow*> windows;
    int16_t tab_index{0};
    int16_t highest_tab_index{0};
};
}  // namespace eros_nodes::RemoteControl