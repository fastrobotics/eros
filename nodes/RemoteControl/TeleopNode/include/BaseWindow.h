#pragma once
#include <curses.h>
#include <eros/Logger.h>

#include <string>

#include "IWindow.h"
#include "RemoteControlUtility.h"
#include "WindowDefinitions.h"
#include "ros/ros.h"
namespace eros_nodes::RemoteControl {
/**
 * @brief Abstract class for Windows that supports some common implementation.
 *
 */
class BaseWindow : public IWindow
{
   public:
    BaseWindow(const std::string _name,
               double start_x_perc,
               double start_y_perc,
               double width_perc,
               double height_perc,
               ros::NodeHandle* nodeHandle,
               std::string robot_namespace,
               eros::Logger* logger,
               uint16_t mainwindow_height,
               uint16_t mainwindow_width)
        : name(_name),
          screen_coord_perc(start_x_perc, start_y_perc, width_perc, height_perc),
          screen_coord_pixel(0, 0, 0, 0),
          nodeHandle(nodeHandle),
          robot_namespace(robot_namespace),
          logger(logger),
          mainwindow_height(mainwindow_height),
          mainwindow_width(mainwindow_width),
          win_(nullptr) {
    }

    virtual ~BaseWindow() {
    }
    std::string get_name() {
        return name;
    }
    bool has_focus() {
        return focused;
    }
    void set_focused(bool cmd_focus) {
        focused = cmd_focus;
    }
    void set_screen_coordinates_pix(ScreenCoordinatePixel coord) {
        screen_coord_pixel = coord;
    }
    ScreenCoordinatePerc get_screen_coordinates_perc() {
        return screen_coord_perc;
    }
    ScreenCoordinatePixel get_screen_coordinates_pixel() {
        return screen_coord_pixel;
    }
    void set_window(WINDOW* win) {
        win_ = win;
    }
    WINDOW* get_window() {
        return win_;
    }
    virtual bool update_window() = 0;

    std::vector<int> get_supported_keys() override {
        return supported_keys;
    }

   protected:
    std::string name;
    ScreenCoordinatePerc screen_coord_perc;
    ScreenCoordinatePixel screen_coord_pixel;
    ros::NodeHandle* nodeHandle;
    std::string robot_namespace;
    eros::Logger* logger;
    uint16_t mainwindow_height;
    uint16_t mainwindow_width;

    double t_ros_time_{0.0};
    bool update(double /*dt*/, double t_ros_time);

    bool focused{false};

    std::vector<int> supported_keys;

   private:
    WINDOW* win_;
};
}  // namespace eros_nodes::RemoteControl