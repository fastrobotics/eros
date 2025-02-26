/*! \file WindowDefinitions.h
 */
#pragma once
#include <eros/eros_Definitions.h>
#include <eros_window/CommonWindowDefinitions.h>

#include <string>
//! Namespace for SystemMonitor
namespace eros_nodes::SystemMonitor {
enum class WindowCommandType {
    UNKNOWN = 0,                 /*!< Uninitialized value. */
    VIEW_DIAGNOSTICS_NODE = 1,   /*!< View Diagnostics for a Node. */
    VIEW_DIAGNOSTICS_SYSTEM = 2, /*!< View Diagnostics for the System. */
    END_OF_LIST = 3              /*!< Last item of list. Used for Range Checks. */
};
/*! \struct ScreenCoordinatePerc
    \brief ScreenCoordinatePerc container.
    */
struct ScreenCoordinatePerc {
    ScreenCoordinatePerc(double start_x, double start_y, double width, double height)
        : start_x_perc(start_x), start_y_perc(start_y), width_perc(width), height_perc(height) {
    }
    double start_x_perc;
    double start_y_perc;
    double width_perc;
    double height_perc;
};
/*! \struct ScreenCoordinatePixel
\brief ScreenCoordinatePixel container.
*/
struct ScreenCoordinatePixel {
    ScreenCoordinatePixel(double start_x, double start_y, double width, double height)
        : start_x_pix(start_x), start_y_pix(start_y), width_pix(width), height_pix(height) {
    }
    uint16_t start_x_pix;
    uint16_t start_y_pix;
    uint16_t width_pix;
    uint16_t height_pix;
};

/**
 * @brief Container for results of a Key Event
 *
 */
struct KeyEventContainer {
    WindowCommand command;
    MessageText message;
};
}  // namespace eros_nodes::SystemMonitor