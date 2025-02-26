/*! \file TeleopNodeProcess.h
 */
#pragma once
#include <curses.h>
#include <eros/BaseNodeProcess.h>

#include "IWindow.h"
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
    bool get_killme() {
        return kill_me;
    }

    // Utility Functions

    // Support Functions
    std::vector<eros::eros_diagnostic::Diagnostic> check_programvariables();

    // Message Functions
    std::vector<eros::eros_diagnostic::Diagnostic> new_commandmsg(eros::command msg);

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
    std::string robot_namespace;
    uint16_t mainwindow_width;
    uint16_t mainwindow_height;

    std::vector<IWindow*> windows;
};
}  // namespace eros_nodes::RemoteControl