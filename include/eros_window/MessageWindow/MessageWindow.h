#pragma once

#include <eros_window/BaseWindow.h>
namespace eros_window {
class MessageWindow : public eros_window::BaseWindow
{
    static constexpr double TIME_TO_SHOW_MESSAGES = 10.0f;  // Seconds
   public:
    static constexpr double START_X_PERC =
        0.0; /*!< What percentage of the screen to put top left corner (X) of window. */
    static constexpr double START_Y_PERC =
        75.0; /*!< What percentage of the screen to put top left corner (Y) of window. */
    static constexpr double WIDTH_PERC =
        100.0; /*!< What percentage of the screen (Width) to draw the window. */
    static constexpr double HEIGHT_PERC =
        7.0; /*!< What percentage of the screen (Height) to draw the window. */
    MessageWindow(ros::NodeHandle* nodeHandle,
                  std::string robot_namespace,
                  eros::Logger* logger,
                  int16_t tab_order,
                  uint16_t mainwindow_height,
                  uint16_t mainwindow_width)
        : BaseWindow("message_window",
                     tab_order,
                     START_X_PERC,
                     START_Y_PERC,
                     WIDTH_PERC,
                     HEIGHT_PERC,
                     nodeHandle,
                     robot_namespace,
                     logger,
                     mainwindow_height,
                     mainwindow_width) {
        // NO Supported Keys
        eros_window::ScreenCoordinatePixel coord_pix =
            eros_window::CommonWindowUtility::convertCoordinate(
                get_screen_coordinates_perc(), mainwindow_width, mainwindow_height);
        WINDOW* win = eros_window::CommonWindowUtility::create_newwin(coord_pix.height_pix,
                                                                      coord_pix.width_pix,
                                                                      coord_pix.start_y_pix,
                                                                      coord_pix.start_x_pix);
        set_screen_coordinates_pix(coord_pix);
        set_window(win);
        wrefresh(win);
    }
    virtual ~MessageWindow();
    bool is_selectable() override {
        return false;
    }
    bool update(double dt, double t_ros_time) override;
    bool new_msg(eros::command_state command_state_msg) override;
    bool new_msg(eros::ArmDisarm::State /* armed_state */) override {  // Not Used
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
    eros_window::KeyEventContainer new_keyevent(int /* key */) override {  // Not Used
        eros_window::KeyEventContainer output;
        return output;
    }
    bool new_MessageTextList(std::vector<eros_window::MessageText> messages);
    bool new_command(std::vector<eros_window::WindowCommand> /* commands*/) override {  // Not Used
        return true;
    }

   private:
    bool update_window();
    void set_message_text(std::string text, eros::Level::Type level);
    void set_message_text(std::string text, eros_window::Color color);
    double timer_showing_message_in_window{0.0};
    std::string message_text;
    eros_window::Color message_text_color{eros_window::Color::NO_COLOR};
};
}  // namespace eros_window