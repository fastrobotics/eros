/*! \file test_nodeWindow.cpp
 */
#include <eros_window/CommonWindowDefinitions.h>
#include <gtest/gtest.h>
#include <stdio.h>

#include "NodeWindow/NodeWindow.h"
using namespace eros_nodes::SystemMonitor;
TEST(BasicTest, Test_Initialization) {
    eros::Logger* logger = new eros::Logger("INFO", "test_node_window");
    NodeWindow SUT(nullptr, "/", logger, 0, 400, 400);
    SUT.set_focused(true);
    // Verify Properties
    EXPECT_EQ(SUT.get_name(), "node_window");
    EXPECT_GT(SUT.get_supported_keys().size(), 0);
    EXPECT_TRUE(SUT.has_focus());
    eros_window::ScreenCoordinatePixel empty_coordinates_pixel(0.0, 0.0, 0.0, 0.0);
    SUT.set_screen_coordinates_pix(empty_coordinates_pixel);
    auto screen_coord_perc = SUT.get_screen_coordinates_perc();
    EXPECT_EQ(screen_coord_perc.start_x_perc, NodeWindow::START_X_PERC);
    EXPECT_EQ(screen_coord_perc.start_y_perc, NodeWindow::START_Y_PERC);
    EXPECT_EQ(screen_coord_perc.width_perc, NodeWindow::WIDTH_PERC);
    EXPECT_EQ(screen_coord_perc.height_perc, NodeWindow::HEIGHT_PERC);
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
        std::vector<eros_window::WindowCommand> commands;
        EXPECT_TRUE(SUT.new_command(commands));
    }
    EXPECT_FALSE(SUT.update(0.0, 0.0));  // Can't update Window, this requires Drawing.
    delete logger;
}
TEST(BasicTest, Test_Keys_DefaultData) {
    ros::NodeHandle nh("~");
    eros::Logger* logger = new eros::Logger("DEBUG", "test_instruction_window");
    NodeWindow SUT(&nh, "/", logger, 0, 400, 400);
    SUT.set_focused(true);
    // Verify Properties
    EXPECT_EQ(SUT.get_name(), "node_window");

    // Check Keys
    for (auto key : SUT.get_supported_keys()) {
        auto result = SUT.new_keyevent(key);
        EXPECT_TRUE(result.message.level <= eros::Level::Type::ERROR);
    }

    delete logger;
}
uint64_t node_firmware_service_rx = 0;
bool node_firmware_service(eros::srv_firmware::Request& /* req */,
                           eros::srv_firmware::Response& /* res */) {
    node_firmware_service_rx++;
    return true;
}
uint64_t node_loggerlevel_service_rx = 0;
bool node_loggerlevel_service(eros::srv_logger_level::Request& /* req */,
                              eros::srv_logger_level::Response& /* res */) {
    node_loggerlevel_service_rx++;
    return true;
}
uint64_t node_change_nodestate_service_rx = 0;
bool node_change_nodestate_service(eros::srv_change_nodestate::Request& /* req */,
                                   eros::srv_change_nodestate::Response& /* res */) {
    node_change_nodestate_service_rx++;
    return true;
}
TEST(BasicTest, Test_Keys_WithNodes) {
    ros::NodeHandle nh("~");
    eros::Logger* logger = new eros::Logger("DEBUG", "test_instruction_window");
    NodeWindow SUT(&nh, "/", logger, 0, 400, 400);
    std::vector<std::string> nodes;
    nodes.push_back("Node1");
    nodes.push_back("Node2");
    nodes.push_back("Node3");
    std::vector<ros::ServiceServer> firmware_srv_list;
    std::vector<ros::ServiceServer> loggerlevel_srv_list;
    std::vector<ros::ServiceServer> change_nodestate_srv_list;
    for (auto node : nodes) {
        std::string srv_firmware_topic = "/" + node + "/srv_firmware";
        ros::ServiceServer node_firmware_srv;
        node_firmware_srv = nh.advertiseService(srv_firmware_topic, &node_firmware_service);
        firmware_srv_list.push_back(node_firmware_srv);
        std::string srv_loggerlevel_topic = "/" + node + "/srv_loggerlevel";
        ros::ServiceServer node_loggerlevel_srv;
        node_loggerlevel_srv =
            nh.advertiseService(srv_loggerlevel_topic, &node_loggerlevel_service);
        loggerlevel_srv_list.push_back(node_loggerlevel_srv);
        std::string srv_change_nodestate_topic = "/" + node + "/srv_nodestate_change";
        ros::ServiceServer node_change_nodestate_srv;
        node_change_nodestate_srv =
            nh.advertiseService(srv_change_nodestate_topic, &node_change_nodestate_service);
        change_nodestate_srv_list.push_back(node_change_nodestate_srv);
    }

    SUT.set_focused(true);
    // Verify Properties
    EXPECT_EQ(SUT.get_name(), "node_window");
    // Feed it Some Data
    for (auto node : nodes) {
        eros::heartbeat heartbeat;
        heartbeat.NodeName = node;
        EXPECT_TRUE(SUT.new_msg(heartbeat));
        EXPECT_TRUE(SUT.new_msg(heartbeat));
    }
    for (auto str : SUT.get_node_info()) { logger->log_info(str); }
    // Check Keys

    for (auto key : SUT.get_supported_keys()) {
        if ((key == eros_window::KEY_1) || (key == eros_window::KEY_2) ||
            (key == eros_window::KEY_3) || (key == eros_window::KEY_4) ||
            (key == eros_window::KEY_5) || (key == eros_window::KEY_6) ||
            (key == eros_window::KEY_7) || (key == eros_window::KEY_8) ||
            (key == eros_window::KEY_9)) {}  // Do Nothing
        else {
            if ((key == eros_window::KEY_l) || (key == eros_window::KEY_L)) {
                auto result = SUT.new_keyevent(key);
                EXPECT_TRUE(result.message.level <= eros::Level::Type::ERROR);
                result = SUT.new_keyevent(eros_window::KEY_3);
                EXPECT_TRUE(result.message.level <= eros::Level::Type::ERROR);
            }
            if ((key == eros_window::KEY_n) || (key == eros_window::KEY_N)) {
                auto result = SUT.new_keyevent(key);
                EXPECT_TRUE(result.message.level <= eros::Level::Type::ERROR);
                result = SUT.new_keyevent(eros_window::KEY_5);
                EXPECT_TRUE(result.message.level <= eros::Level::Type::ERROR);
            }
            else {
                auto result = SUT.new_keyevent(key);
                EXPECT_TRUE(result.message.level <= eros::Level::Type::ERROR);
            }
        }
    }
    sleep(5);
    EXPECT_GT(node_firmware_service_rx, 0);
    EXPECT_GT(node_loggerlevel_service_rx, 0);
    EXPECT_GT(node_change_nodestate_service_rx, 0);

    delete logger;
}
int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "test_nodeWindow");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    int ret = RUN_ALL_TESTS();
    spinner.stop();
    ros::shutdown();
    return ret;
}