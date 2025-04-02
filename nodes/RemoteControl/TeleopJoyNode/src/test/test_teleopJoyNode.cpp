#include <gtest/gtest.h>
#include <ros/ros.h>

#include "TeleopJoyNode.h"

using namespace eros;

uint64_t heartbeat_count = 0;
uint64_t armedstate_count = 0;
void heartbeat_Callback(const eros::heartbeat& msg) {
    (void)msg;
    heartbeat_count++;
}
TEST(TeleopJoyNode, TestTeleopJoyNodeBasic) {
    ros::NodeHandle nh("~");
    Logger* logger = new Logger("NOTICE", "test_TeleopJoyNode");

    std::string heartbeat_topic = "/test/teleop_joy_node/heartbeat";
    ros::Subscriber sub_heartbeat = nh.subscribe(heartbeat_topic, 100, &heartbeat_Callback);
    sleep(5.0);
    EXPECT_NE(ros::topic::waitForMessage<eros::heartbeat>(heartbeat_topic, ros::Duration(10)),
              nullptr);
    EXPECT_EQ(1, sub_heartbeat.getNumPublishers());
    usleep(1.0 * 1000000.0);  // Wait for Teleop Joy Node to Start.
    EXPECT_TRUE(heartbeat_count > 0);

    // Let Node run for a while
    sleep(1.0);

    delete logger;
}
TEST(TeleopJoyNode, TestTeleopJoyNodeCommand) {
    ros::NodeHandle nh("~");
    Logger* logger = new Logger("NOTICE", "test_TeleopJoyNode");

    std::string system_command_topic = "/test/SystemCommand/";

    ros::Publisher pub_command = nh.advertise<eros::command>(system_command_topic, 1);
    sleep(5.0);
    EXPECT_EQ(1, pub_command.getNumSubscribers());
    eros::command new_command;
    pub_command.publish(new_command);
    sleep(1.0);

    delete logger;
}
TEST(TeleopJoyNode, TestTeleopJoyNodeJoyMsg) {
    ros::NodeHandle nh("~");
    Logger* logger = new Logger("NOTICE", "test_TeleopJoyNode");

    std::string joy_topic = "/test/joy";

    ros::Publisher pub_joy = nh.advertise<sensor_msgs::Joy>(joy_topic, 1);
    sleep(5.0);
    EXPECT_EQ(1, pub_joy.getNumSubscribers());
    sensor_msgs::Joy new_joy;
    pub_joy.publish(new_joy);
    sleep(1.0);

    delete logger;
}
int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "test_TeleopJoyNode");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    int ret = RUN_ALL_TESTS();
    spinner.stop();
    ros::shutdown();
    return ret;
}