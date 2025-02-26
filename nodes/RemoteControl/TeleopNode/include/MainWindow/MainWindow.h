#pragma once
#include <mutex>

#include "BaseWindow.h"
namespace eros_nodes::RemoteControl {

/**
 * @brief A Window to render Device Information
 *
 */
class MainWindow : public BaseWindow
{
   public:
    static constexpr double START_X_PERC =
        55.0; /*!< What percentage of the screen to put top left corner (X) of window. */
    static constexpr double START_Y_PERC =
        80.0; /*!< What percentage of the screen to put top left corner (Y) of window. */
    static constexpr double WIDTH_PERC =
        45.0; /*!< What percentage of the screen (Width) to draw the window. */
    static constexpr double HEIGHT_PERC =
        20.0; /*!< What percentage of the screen (Height) to draw the window. */
    MainWindow(ros::NodeHandle* nodeHandle,
               std::string robot_namespace,
               eros::Logger* logger,
               uint16_t mainwindow_height,
               uint16_t mainwindow_width)
        : BaseWindow("main_window",
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

        ScreenCoordinatePixel coord_pix = RemoteControlUtility::convertCoordinate(
            get_screen_coordinates_perc(), mainwindow_width, mainwindow_height);
        WINDOW* win = RemoteControlUtility::create_newwin(coord_pix.height_pix,
                                                          coord_pix.width_pix,
                                                          coord_pix.start_y_pix,
                                                          coord_pix.start_x_pix);
        set_screen_coordinates_pix(coord_pix);
        set_window(win);
        wrefresh(win);
    }
    virtual ~MainWindow();
    bool update(double dt, double t_ros_time) override;
    KeyEventContainer new_keyevent(int /* key */) override {  // Not Used
        KeyEventContainer output;
        return output;
    }
    std::string pretty();

   private:
    bool update_window();
};
}  // namespace eros_nodes::RemoteControl