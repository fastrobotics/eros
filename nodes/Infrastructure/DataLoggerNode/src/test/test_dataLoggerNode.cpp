#include <actionlib/client/simple_action_client.h>
#include <eros/system_commandAction.h>
#include <gtest/gtest.h>
#include <ros/ros.h>

#include "DataLoggerNode.h"
std::string robot_namespace = "/test/";
std::string unittest_nodename = "datalogger_node";
using namespace eros;
typedef actionlib::SimpleActionClient<eros::system_commandAction> CommandActionClient;
uint64_t heartbeat_count = 0;
eros::heartbeat latest_heartbeat;
void heartbeat_Callback(const eros::heartbeat& msg) {
    latest_heartbeat = msg;
    heartbeat_count++;
}
TEST(DataLoggerNode, TestDataLoggerNode) {
    ros::NodeHandle nh("~");
    Logger* logger = new Logger("DEBUG", "test_dataLoggerNode");
    logger->enable_ROS_logger();
    std::string heartbeat_topic = "/test/datalogger_node/heartbeat";
    ros::Subscriber sub = nh.subscribe(heartbeat_topic, 100, &heartbeat_Callback);
    sleep(5.0);
    EXPECT_NE(ros::topic::waitForMessage<eros::heartbeat>(heartbeat_topic, ros::Duration(10)),
              nullptr);
    EXPECT_EQ(1, sub.getNumPublishers());
    usleep(1.0 * 1000000.0);  // Wait for Master Node to Start.
    EXPECT_TRUE(heartbeat_count > 0);

    logger->log_warn("Testing Unsupported Commands...");
    {  // System Command not currently supported by Node.
        CommandActionClient client(robot_namespace + "SystemCommandAction", true);
        EXPECT_TRUE(client.waitForServer());
        eros::system_commandGoal cmd;
        client.sendGoal(cmd);
        client.waitForResult(ros::Duration(1.0));
        EXPECT_EQ(client.getState(), actionlib::SimpleClientGoalState::ABORTED);
    }
    {
        ros::Publisher command_pub =
            nh.advertise<eros::command>(robot_namespace + "SystemCommand", 20);
        sleep(1);
        EXPECT_EQ(1, command_pub.getNumSubscribers());
        eros::command snapshot_command;
        command_pub.publish(snapshot_command);
        sleep(1.0);
    }
    logger->log_notice("Basic Test of Node State Change...");
    {
        ros::ServiceClient client = nh.serviceClient<eros::srv_change_nodestate>(
            robot_namespace + unittest_nodename + "/srv_nodestate_change");
        eros::srv_change_nodestate req;
        req.request.RequestedNodeState = "PAUSED";
        EXPECT_TRUE(client.call(req));
        sleep(1);
        EXPECT_EQ((uint8_t)Node::State::PAUSED, latest_heartbeat.NodeState);
        req.request.RequestedNodeState = "RUNNING";
        EXPECT_TRUE(client.call(req));
        sleep(1);
        EXPECT_EQ((uint8_t)Node::State::RUNNING, latest_heartbeat.NodeState);

        req.request.RequestedNodeState = "RESET";
        EXPECT_TRUE(client.call(req));
        sleep(1);
        EXPECT_EQ((uint8_t)Node::State::RUNNING,
                  latest_heartbeat.NodeState);  // Node should automatically change state to Running
    }

    logger->log_notice("Test Snapshot");
    {
        ros::Publisher snapshot_trigger_pub =
            nh.advertise<std_msgs::Empty>(robot_namespace + "snapshot_trigger", 20);
        sleep(1);
        EXPECT_EQ(1, snapshot_trigger_pub.getNumSubscribers());
        std_msgs::Empty trigger;
        snapshot_trigger_pub.publish(trigger);
        sleep(1);
    }
    logger->log_warn("Allow time for other functions to get called by Node.");
    sleep(10);

    delete logger;
}
int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "test_dataLoggerNode");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    int ret = RUN_ALL_TESTS();
    spinner.stop();
    ros::shutdown();
    return ret;
}