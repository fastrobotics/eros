#pragma once
#include <eros_window/BaseWindow.h>
#include <geometry_msgs/Twist.h>

#include <mutex>
namespace eros_nodes::RemoteControl {

/**
 * @brief A Window to render Device Information
 *
 */
class MainWindow : public eros_window::BaseWindow
{
   public:
    static constexpr double START_X_PERC =
        0.0; /*!< What percentage of the screen to put top left corner (X) of window. */
    static constexpr double START_Y_PERC =
        15.0; /*!< What percentage of the screen to put top left corner (Y) of window. */
    static constexpr double WIDTH_PERC =
        66.0; /*!< What percentage of the screen (Width) to draw the window. */
    static constexpr double HEIGHT_PERC =
        60.0; /*!< What percentage of the screen (Height) to draw the window. */
    MainWindow(ros::NodeHandle* nodeHandle,
               std::string robot_namespace,
               eros::Logger* logger,
               int16_t tab_order,
               uint16_t mainwindow_height,
               uint16_t mainwindow_width)
        : BaseWindow("main_window",
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
        supported_keys.push_back(KEY_UP);
        supported_keys.push_back(KEY_DOWN);
        supported_keys.push_back(KEY_LEFT);
        supported_keys.push_back(KEY_RIGHT);

        eros_window::ScreenCoordinatePixel coord_pix =
            eros_window::CommonWindowUtility::convertCoordinate(
                get_screen_coordinates_perc(), mainwindow_width, mainwindow_height);
        WINDOW* win = eros_window::CommonWindowUtility::create_newwin(coord_pix.height_pix,
                                                                      coord_pix.width_pix,
                                                                      coord_pix.start_y_pix,
                                                                      coord_pix.start_x_pix);
        std::string commandvel_per_topic = robot_namespace + "cmd_vel_perc";
        cmd_vel_pub = nodeHandle->advertise<geometry_msgs::Twist>(commandvel_per_topic, 1);
        cmd_vel_perc.linear.x = 0.0;
        cmd_vel_perc.angular.z = 0.0;
        set_screen_coordinates_pix(coord_pix);
        set_window(win);
        wrefresh(win);
    }
    virtual ~MainWindow();
    bool update(double dt, double t_ros_time) override;
    bool is_selectable() override {
        return false;
    }
    bool new_msg(eros::ArmDisarm::State armed_state) override;
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
    bool new_command(std::vector<eros_window::WindowCommand> /* commands */) override {  // Not Used
        return true;
    }
    eros_window::KeyEventContainer new_keyevent(int key) override;
    std::string pretty();

   private:
    bool update_window();
    ros::Publisher cmd_vel_pub;
    geometry_msgs::Twist cmd_vel_perc;
    eros::ArmDisarm::State armed_state_;
};
}  // namespace eros_nodes::RemoteControl