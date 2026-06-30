#pragma once
#include <eros_window/BaseWindow.h>
namespace eros_nodes::RemoteControl {
class InstructionWindow : public eros_window::BaseWindow
{
   public:
    enum class InstructionMode {
        NODE = 0,
    };
    static constexpr double START_X_PERC =
        30.0; /*!< What percentage of the screen to put top left corner (X) of window. */
    static constexpr double START_Y_PERC =
        80.0; /*!< What percentage of the screen to put top left corner (Y) of window. */
    static constexpr double WIDTH_PERC =
        25.0; /*!< What percentage of the screen (Width) to draw the window. */
    static constexpr double HEIGHT_PERC =
        20.0; /*!< What percentage of the screen (Height) to draw the window. */
    InstructionWindow(ros::NodeHandle* nodeHandle,
                      std::string robot_namespace,
                      eros::Logger* logger,
                      int16_t tab_order,
                      uint16_t mainwindow_height,
                      uint16_t mainwindow_width,
                      ros::Publisher command_pub)
        : BaseWindow("instruction_window",
                     tab_order,
                     START_X_PERC,
                     START_Y_PERC,
                     WIDTH_PERC,
                     HEIGHT_PERC,
                     nodeHandle,
                     robot_namespace,
                     logger,
                     mainwindow_height,
                     mainwindow_width),
          command_pub(command_pub) {
        supported_keys.push_back(eros_window::KEY_esc);
        supported_keys.push_back(eros_window::KEY_space);
        supported_keys.push_back(eros_window::KEY_s);
        supported_keys.push_back(eros_window::KEY_S);
        supported_keys.push_back(eros_window::KEY_c);
        supported_keys.push_back(eros_window::KEY_C);
        eros_window::ScreenCoordinatePixel coord_pix =
            eros_window::CommonWindowUtility::convertCoordinate(
                get_screen_coordinates_perc(), mainwindow_width, mainwindow_height);
        WINDOW* win = eros_window::CommonWindowUtility::create_newwin(coord_pix.height_pix,
                                                                      coord_pix.width_pix,
                                                                      coord_pix.start_y_pix,
                                                                      coord_pix.start_x_pix);
        set_screen_coordinates_pix(coord_pix);
        set_window(win);

        std::string str = "Instructions:";
        keypad(win, TRUE);
        mvwprintw(win, 1, 1, str.c_str());
        std::string dashed(coord_pix.width_pix - 2, '-');
        mvwprintw(win, 2, 1, dashed.c_str());
        wrefresh(win);
    }
    virtual ~InstructionWindow();
    bool is_selectable() override {
        return false;
    }
    bool update(double dt, double t_ros_time) override;
    bool new_msg(eros::ArmDisarm::State armed_state) override {
        current_armed_state = armed_state;
        return true;
    }
    bool new_msg(eros::heartbeat /* heartbeat_msg */) override {  // Not Used
        return true;
    }
    bool new_msg(eros::resource /*resource_msg*/) override {  // Not Used
        return true;
    }
    bool new_msg(eros::loadfactor /*loadfactor_msg*/) override {  // Not Used
        return true;
    }
    bool new_msg(eros::command_state /* command_state_msg */) override {  // Not Used
        return true;
    }
    void set_InstructionMode(InstructionMode cmd_mode) {
        instruction_mode = cmd_mode;
    }
    eros_window::KeyEventContainer new_keyevent(int key) override;
    bool new_command(std::vector<eros_window::WindowCommand> /* commands*/) override {  // Not Used
        return true;
    }

   private:
    bool update_window();
    InstructionMode instruction_mode{InstructionMode::NODE};
    ros::Publisher command_pub;
    eros::ArmDisarm::State current_armed_state;
};
}  // namespace eros_nodes::RemoteControl