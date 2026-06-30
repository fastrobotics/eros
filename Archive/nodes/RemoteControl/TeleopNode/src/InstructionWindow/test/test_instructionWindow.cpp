/*! \file test_instructionwindow.cpp
 */
#include <eros_window/CommonWindowDefinitions.h>
#include <gtest/gtest.h>
#include <stdio.h>

#include "InstructionWindow/InstructionWindow.h"
using namespace eros_nodes::RemoteControl;
uint64_t command_receive_counter = 0;
void command_Callback(const eros::command& /* msg */) {
    command_receive_counter++;
}
TEST(BasicTest, Test_Initialization) {
    ros::NodeHandle nh("~");
    ros::Publisher command_pub = nh.advertise<eros::command>("/SystemCommand", 1);
    eros::Logger* logger = new eros::Logger("INFO", "test_instruction_window");
    InstructionWindow SUT(&nh, "/", logger, 0, 400, 400, command_pub);
    // Verify Properties
    EXPECT_EQ(SUT.get_name(), "instruction_window");
    EXPECT_GT(SUT.get_supported_keys().size(), 0);  // Has multiple Supported Keys
    EXPECT_FALSE(SUT.has_focus());
    SUT.set_focused(false);
    eros_window::ScreenCoordinatePixel empty_coordinates_pixel(0.0, 0.0, 0.0, 0.0);
    SUT.set_screen_coordinates_pix(empty_coordinates_pixel);
    auto screen_coord_perc = SUT.get_screen_coordinates_perc();
    EXPECT_EQ(screen_coord_perc.start_x_perc, InstructionWindow::START_X_PERC);
    EXPECT_EQ(screen_coord_perc.start_y_perc, InstructionWindow::START_Y_PERC);
    EXPECT_EQ(screen_coord_perc.width_perc, InstructionWindow::WIDTH_PERC);
    EXPECT_EQ(screen_coord_perc.height_perc, InstructionWindow::HEIGHT_PERC);
    auto screen_coord_pixel = SUT.get_screen_coordinates_pixel();
    EXPECT_EQ(screen_coord_pixel.start_x_pix, 0);
    EXPECT_EQ(screen_coord_pixel.start_y_pix, 0);
    EXPECT_EQ(screen_coord_pixel.width_pix, 0);
    EXPECT_EQ(screen_coord_pixel.height_pix, 0);
    logger->log_debug(SUT.pretty());
    EXPECT_EQ(SUT.get_tab_order(), 0);
    SUT.set_window_records_are_selectable(false);
    EXPECT_FALSE(SUT.get_window_records_are_selectable());
    EXPECT_EQ(SUT.get_selected_record(), 0);
    EXPECT_FALSE(SUT.is_selectable());

    // Verify Unsupported Commands
    {
        eros::ArmDisarm::State msg;
        EXPECT_TRUE(SUT.new_msg(msg));
    }
    {
        eros::heartbeat msg;
        EXPECT_TRUE(SUT.new_msg(msg));
    }
    {
        eros::command_state msg;
        EXPECT_TRUE(SUT.new_msg(msg));
    }
    {
        eros::resource msg;
        EXPECT_TRUE(SUT.new_msg(msg));
    }
    {
        eros::loadfactor msg;
        EXPECT_TRUE(SUT.new_msg(msg));
    }

    {
        std::vector<eros_window::WindowCommand> commands;
        EXPECT_TRUE(SUT.new_command(commands));
    }
    EXPECT_FALSE(SUT.update(0.0, 0.0));  // Can't update Window, this requires Drawing.
    delete logger;
}

TEST(BasicTest, Test_Keys) {
    command_receive_counter = 0;
    ros::NodeHandle nh("~");
    ros::Publisher command_pub = nh.advertise<eros::command>("/SystemCommand", 1);
    ros::Subscriber command_sub = nh.subscribe("/SystemCommand", 100, &command_Callback);
    eros::Logger* logger = new eros::Logger("INFO", "test_instruction_window");
    InstructionWindow SUT(&nh, "/", logger, 0, 400, 400, command_pub);
    // Verify Properties
    EXPECT_EQ(SUT.get_name(), "instruction_window");

    // Check Keys
    for (auto key : SUT.get_supported_keys()) {
        if (key == eros_window::KEY_space) {  // Arm/Disarm Key result depends on state
            eros::ArmDisarm::State armed_state;
            armed_state.state = eros::ArmDisarm::Type::ARMED;
            EXPECT_TRUE(SUT.new_msg(armed_state));
            auto result = SUT.new_keyevent(key);
            EXPECT_TRUE(result.message.level < eros::Level::Type::WARN);
            armed_state.state = eros::ArmDisarm::Type::DISARMED;
            EXPECT_TRUE(SUT.new_msg(armed_state));
            result = SUT.new_keyevent(key);
            EXPECT_TRUE(result.message.level < eros::Level::Type::WARN);
            armed_state.state = eros::ArmDisarm::Type::DISARMED_CANNOTARM;
            EXPECT_TRUE(SUT.new_msg(armed_state));
            result = SUT.new_keyevent(key);
            EXPECT_TRUE(result.message.level >= eros::Level::Type::WARN);
        }
        else if ((key == eros_window::KEY_C) ||
                 (key == eros_window::KEY_c)) {  // Snapshot Clear should indicate a Warning
            auto result = SUT.new_keyevent(key);
            EXPECT_TRUE(result.message.level == eros::Level::Type::WARN);
        }
        else {
            auto result = SUT.new_keyevent(key);
            EXPECT_TRUE(result.message.level < eros::Level::Type::WARN);
        }
    }
    sleep(5);  // Give a little time for ROS to send messages
    EXPECT_TRUE(command_receive_counter > 0);
    delete logger;
}
int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "test_instructionWindow");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    int ret = RUN_ALL_TESTS();
    spinner.stop();
    ros::shutdown();
    return ret;
}