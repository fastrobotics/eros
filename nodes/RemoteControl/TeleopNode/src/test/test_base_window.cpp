/*! \file test_base_window.cpp
 */
#include <eros_window/BaseWindow.h>
#include <eros_window/CommonWindowDefinitions.h>namespace eros_nodes::SystemMonitor {
#include <gtest/gtest.h>
#include <stdio.h>
class TesterBaseWindow : public eros_window::BaseWindow
{
   public:
    TesterBaseWindow()
        : BaseWindow("tester_window", 0, 0.0, 0.0, 0.0, 0.0, nullptr, "", nullptr, 0, 0) {
    }
    virtual ~TesterBaseWindow() {
    }
    bool is_selectable() override {
        return false;
    }
    bool update(double dt, double t_ros_time) override {
        bool status = BaseWindow::update(dt, t_ros_time);
        if (status == false) {
            return false;
        }
        return true;
    }
    bool new_msg(eros::ArmDisarm::State /* armed_state */) {
        return false;
    }
    bool new_msg(eros::heartbeat /* heartbeat_msg */) override {
        return false;
    }
    bool new_msg(eros::resource /* resource_msg */) override {
        return false;
    }
    bool new_msg(eros::loadfactor /* loadfactor_msg */) override {
        return false;
    }
    bool new_msg(eros::command_state /* command_state_msg */) override {
        return false;
    }
    eros_window::KeyEventContainer new_keyevent(int key) override {
        eros_window::KeyEventContainer output;
        if (key == KEY_UP) {
            decrement_selected_record();
        }
        else if (key == KEY_DOWN) {
            increment_selected_record();
        }
        else if (key == eros_window::KEY_5) {
            for (int i = 0; i < 5; ++i) { update_record_count(i); }
        }
        return output;
    }
    bool new_command(std::vector<eros_window::WindowCommand> /* commands */) override {
        return false;
    }
    bool update_window() {
        return false;
    }

   private:
};
}  // namespace eros_nodes::SystemMonitor
using namespace eros_nodes::SystemMonitor;
TEST(BasicTest, Test_DefaultInitialization) {
    TesterBaseWindow SUT;
    EXPECT_EQ(SUT.get_name(), "tester_window");
    EXPECT_FALSE(SUT.has_focus());
    SUT.set_focused(false);
    eros_window::ScreenCoordinatePixel empty_coordinates_pixel(0.0, 0.0, 0.0, 0.0);
    SUT.set_screen_coordinates_pix(empty_coordinates_pixel);
    auto screen_coord_perc = SUT.get_screen_coordinates_perc();
    EXPECT_EQ(screen_coord_perc.start_x_perc, 0);
    EXPECT_EQ(screen_coord_perc.start_y_perc, 0);
    EXPECT_EQ(screen_coord_perc.width_perc, 0);
    EXPECT_EQ(screen_coord_perc.height_perc, 0);
    auto screen_coord_pixel = SUT.get_screen_coordinates_pixel();
    EXPECT_EQ(screen_coord_pixel.start_x_pix, 0);
    EXPECT_EQ(screen_coord_pixel.start_y_pix, 0);
    EXPECT_EQ(screen_coord_pixel.width_pix, 0);
    EXPECT_EQ(screen_coord_pixel.height_pix, 0);
    printf("%s\n", SUT.pretty().c_str());
    EXPECT_FALSE(SUT.update_window());
    EXPECT_EQ(SUT.get_tab_order(), 0);
    SUT.set_window_records_are_selectable(false);
    EXPECT_FALSE(SUT.get_window_records_are_selectable());
    EXPECT_EQ(SUT.get_selected_record(), 0);
    auto output = SUT.new_keyevent(KEY_UP);
    output = SUT.new_keyevent(KEY_DOWN);
    output = SUT.new_keyevent(eros_window::KEY_5);
    EXPECT_TRUE(SUT.update(0.0, 0.0));
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}