#pragma once
#include <curses.h>

#include "CommonWindowDefinitions.h"
namespace eros_window {
/**
 * @brief Various utility functions for any Window UI
 *
 */
class CommonWindowUtility
{
   public:
    static WINDOW* create_newwin(int height, int width, int starty, int startx);
    static ScreenCoordinatePixel convertCoordinate(ScreenCoordinatePerc coord_perc,
                                                   uint16_t width_pix,
                                                   uint16_t height_pix);
};

}  // namespace eros_window