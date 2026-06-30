/*! \file test_diagnosticswindow.cpp
 */
// Add Gtest First
#include <gtest/gtest.h>

// Add the rest
#include <eros_window/CommonWindowDefinitions.h>
#include <eros_window/DiagnosticsWindow/DiagnosticsWindow.h>
#include <stdio.h>
using namespace eros_window;
TEST(BasicTest, Test_Initialization) {
    printf("Starting-1\n");
    eros::Logger* logger = new eros::Logger("INFO", "test_diagnostics_window");
    DiagnosticsWindow SUT(nullptr, "/", logger, 0, 400, 400);
    // Verify Properties
    EXPECT_EQ(SUT.get_name(), "diagnostics_window");
    EXPECT_EQ(SUT.get_supported_keys().size(), 0);  // NO Supported Keys
    EXPECT_FALSE(SUT.has_focus());
    SUT.set_focused(false);
    eros_window::ScreenCoordinatePixel empty_coordinates_pixel(0.0, 0.0, 0.0, 0.0);
    SUT.set_screen_coordinates_pix(empty_coordinates_pixel);
    auto screen_coord_perc = SUT.get_screen_coordinates_perc();
    EXPECT_EQ(screen_coord_perc.start_x_perc, DiagnosticsWindow::START_X_PERC);
    EXPECT_EQ(screen_coord_perc.start_y_perc, DiagnosticsWindow::START_Y_PERC);
    EXPECT_EQ(screen_coord_perc.width_perc, DiagnosticsWindow::WIDTH_PERC);
    EXPECT_EQ(screen_coord_perc.height_perc, DiagnosticsWindow::HEIGHT_PERC);
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
    EXPECT_TRUE(SUT.is_selectable());

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
        int key = -1;
        SUT.new_keyevent(key);
    }
    {
        std::vector<eros_window::WindowCommand> commands;
        EXPECT_TRUE(SUT.new_command(commands));
    }
    EXPECT_FALSE(SUT.update(0.0, 0.0));  // Can't update Window, this requires Drawing.
    delete logger;
}
uint64_t system_diagnostic_service_rx = 0;
bool system_diagnostics_service(eros::srv_get_diagnostics::Request& /* req */,
                                eros::srv_get_diagnostics::Response& /* res */) {
    system_diagnostic_service_rx++;
    return true;
}
TEST(BasicTest, Test_OperationSystemDiagnostics) {
    ros::NodeHandle nh("~");
    std::string srv_system_diagnostics_topic = "/srv_system_diagnostics";
    ros::ServiceServer system_diagnostics_srv;
    system_diagnostics_srv =
        nh.advertiseService(srv_system_diagnostics_topic, &system_diagnostics_service);
    eros::Logger* logger = new eros::Logger("INFO", "test_diagnostics_window");
    DiagnosticsWindow SUT(&nh, "/", logger, 0, 400, 400);
    EXPECT_EQ(SUT.get_name(), "diagnostics_window");
    {
        std::vector<eros_window::WindowCommand> commands;
        eros_window::WindowCommand command;
        command.type = eros_window::WindowCommandType::VIEW_DIAGNOSTICS_SYSTEM;
        commands.push_back(command);
        EXPECT_TRUE(SUT.new_command(commands));
    }
    double current_time = 0.0;
    double delta_time = 0.1;
    while (current_time < 30) {
        current_time += delta_time;
        EXPECT_FALSE(SUT.update(delta_time, current_time));
        usleep(delta_time * 1000000);
        if (system_diagnostic_service_rx > 5) {
            break;
        }
    }
    EXPECT_TRUE(system_diagnostic_service_rx > 0);

    delete logger;
}
uint64_t node_diagnostic_service_rx = 0;
bool node_diagnostics_service(eros::srv_get_diagnostics::Request& /* req */,
                              eros::srv_get_diagnostics::Response& /* res */) {
    node_diagnostic_service_rx++;
    return true;
}
TEST(BasicTest, Test_OperationNodeDiagnostics) {
    std::string node_name = "diagnostics_window";
    ros::NodeHandle nh("~");
    std::string srv_node_diagnostics_topic = "/" + node_name + "/srv_diagnostics";
    ros::ServiceServer system_diagnostics_srv;
    system_diagnostics_srv =
        nh.advertiseService(srv_node_diagnostics_topic, &node_diagnostics_service);
    eros::Logger* logger = new eros::Logger("INFO", "test_diagnostics_window");
    DiagnosticsWindow SUT(&nh, "/", logger, 0, 400, 400);
    EXPECT_EQ(SUT.get_name(), "diagnostics_window");
    {
        std::vector<eros_window::WindowCommand> commands;
        eros_window::WindowCommand command;
        command.type = eros_window::WindowCommandType::VIEW_DIAGNOSTICS_NODE;
        command.option = "/" + node_name;
        commands.push_back(command);
        EXPECT_TRUE(SUT.new_command(commands));
    }
    double current_time = 0.0;
    double delta_time = 0.1;
    while (current_time < 30) {
        current_time += delta_time;
        EXPECT_FALSE(SUT.update(delta_time, current_time));
        usleep(delta_time * 1000000);
        if (node_diagnostic_service_rx > 5) {
            break;
        }
    }
    EXPECT_TRUE(node_diagnostic_service_rx > 0);

    delete logger;
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "test_diagnosticsWindow");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    int ret = RUN_ALL_TESTS();
    spinner.stop();
    ros::shutdown();
    return ret;
}