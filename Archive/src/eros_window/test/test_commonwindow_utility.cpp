/*! \file test_commonwindow_utility.cpp
 */
#include <gtest/gtest.h>

//#include <eros_window/CommonWindowDefinitions.h>
#include <eros_window/CommonWindowUtility.h>
#include <stdio.h>
using namespace eros_window;
TEST(BasicTest, Test_Convert) {
    uint16_t width_pix = 100;
    uint16_t height_pix = 100;
    ScreenCoordinatePerc coord_perc(0.0, 0.0, 100.0, 100.0);
    auto coord_pixel = CommonWindowUtility::convertCoordinate(coord_perc, width_pix, height_pix);
    EXPECT_EQ(coord_pixel.start_x_pix, 0);
    EXPECT_EQ(coord_pixel.start_y_pix, 0);
    EXPECT_EQ(coord_pixel.width_pix, width_pix);
    EXPECT_EQ(coord_pixel.height_pix, height_pix);
}
TEST(BasicTest, Test_NewWindow) {
    auto window = CommonWindowUtility::create_newwin(100, 100, 0, 0);
    EXPECT_TRUE(window == nullptr);  // This won't work in unit tests.
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}