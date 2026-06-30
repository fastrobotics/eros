/*! \file test_systemmonitor_node.cpp
 */
// clang-format off
#include <gtest/gtest.h>
#include "../TeleopNode.h"
#include <stdio.h>
using namespace eros;
using namespace eros_nodes::RemoteControl;
TEST(SnapshotNode, TestMaster) {
    ros::NodeHandle nh("~");
    Logger* logger = new Logger("DEBUG", "test_teleop_node");
    EXPECT_TRUE(true);
    delete logger;
}
int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "test_teleop_node");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    int ret = RUN_ALL_TESTS();
    spinner.stop();
    ros::shutdown();
    return ret;
}