#include <gtest/gtest.h>
#include <ros/ros.h>

#include "SnapshotNode.h"

using namespace eros;
uint64_t heartbeat_count = 0;
uint64_t commandstate_count = 0;
eros::command_state commandstate_msg;
void heartbeat_Callback(const eros::heartbeat& msg) {
    (void)msg;
    heartbeat_count++;
}
void commandstate_Callback(const eros::command_state& msg) {
    commandstate_count++;
    commandstate_msg = msg;
}
TEST(SnapshotNode, TestMaster) {
    ros::NodeHandle nh("~");
    Logger* logger = new Logger("DEBUG", "test_SnapshotNode");
    logger->enable_ROS_logger();
    std::string heartbeat_topic = "/test/snapshot_node/heartbeat";
    ros::Subscriber sub = nh.subscribe(heartbeat_topic, 100, &heartbeat_Callback);
    sleep(5.0);
    EXPECT_NE(ros::topic::waitForMessage<eros::heartbeat>(heartbeat_topic, ros::Duration(10)),
              nullptr);
    EXPECT_EQ(1, sub.getNumPublishers());
    ros::Subscriber commandstate_sub =
        nh.subscribe("/test/SystemCommandState", 100, &commandstate_Callback);
    ros::Publisher command_pub = nh.advertise<eros::command>("/test/SystemCommand", 20);
    usleep(1.0 * 1000000.0);  // Wait for Snapshot Node to Start.
    EXPECT_TRUE(heartbeat_count > 0);

    eros::command snapshot_command;
    logger->log_notice("Testing Clearing of Snapshots.");
    snapshot_command.Command = (uint8_t)eros::Command::Type::GENERATE_SNAPSHOT;
    snapshot_command.Option1 = (uint8_t)eros::Command::GenerateSnapshot_Option1::CLEAR_SNAPSHOTS;
    command_pub.publish(snapshot_command);
    usleep(5.0 * 1000000.0);
    EXPECT_TRUE(commandstate_count > 0);
    EXPECT_TRUE(commandstate_msg.CurrentCommand.Command ==
                (uint8_t)eros::Command::Type::GENERATE_SNAPSHOT);
    EXPECT_TRUE(commandstate_msg.CurrentCommand.Option1 ==
                (uint8_t)eros::Command::GenerateSnapshot_Option1::CLEAR_SNAPSHOTS);
    EXPECT_TRUE(commandstate_msg.diag.DiagnosticMessage ==
                (uint8_t)eros_diagnostic::Message::NOERROR);
    commandstate_count = 0;
    logger->log_notice("Testing Generation of Snapshot");
    snapshot_command.Command = (uint8_t)eros::Command::Type::GENERATE_SNAPSHOT;
    snapshot_command.Option1 = (uint8_t)eros::Command::GenerateSnapshot_Option1::RUN_MASTER;
    command_pub.publish(snapshot_command);
    usleep(5.0 * 1000000.0);
    EXPECT_TRUE(commandstate_count > 0);
    EXPECT_TRUE(commandstate_msg.CurrentCommand.Command ==
                (uint8_t)eros::Command::Type::GENERATE_SNAPSHOT);
    EXPECT_TRUE(commandstate_msg.CurrentCommand.Option1 ==
                (uint8_t)eros::Command::GenerateSnapshot_Option1::RUN_MASTER);
    logger->log_info(commandstate_msg.diag.Description);
    EXPECT_TRUE(commandstate_msg.diag.DiagnosticMessage ==
                (uint8_t)eros_diagnostic::Message::NOERROR);
    delete logger;
}
int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "test_snapshotNode");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    int ret = RUN_ALL_TESTS();
    spinner.stop();
    ros::shutdown();
    return ret;
}