#pragma once
#include <eros/command_state.h>
#include <eros/eros_Definitions.h>
#include <eros/heartbeat.h>
#include <eros/loadfactor.h>
#include <eros/resource.h>
#include <eros_diagnostic/Diagnostic.h>
#include <eros_diagnostic/DiagnosticUtility.h>

#include "CommonWindowDefinitions.h"
namespace eros_window {
/**
 * @brief Interface class for generic Windows
 *
 */
class IWindow
{
   public:
    virtual ~IWindow() {
    }
    virtual std::string get_name() = 0;
    virtual bool update(double dt, double t_ros_time) = 0;
    virtual bool new_msg(eros::ArmDisarm::State armed_state) = 0;
    virtual bool new_msg(eros::heartbeat heartbeat_msg) = 0;
    virtual bool new_msg(eros::resource resource_msg) = 0;
    virtual bool new_msg(eros::loadfactor loadfactor_msg) = 0;
    virtual bool new_msg(eros::command_state command_state_msg) = 0;

    virtual bool new_command(std::vector<WindowCommand> commands) = 0;
    virtual bool has_focus() = 0;
    /*! \brief Set focus.  Key events will be passed to any window that has focus.  Windows that
     * aren't focused will not receive any key events.
     */
    virtual void set_focused(bool cmd_focus) = 0;
    virtual bool is_selectable() = 0;

    virtual int16_t get_tab_order() = 0;

    virtual std::vector<int> get_supported_keys() = 0;
    virtual KeyEventContainer new_keyevent(int key) = 0;
};
}  // namespace eros_window