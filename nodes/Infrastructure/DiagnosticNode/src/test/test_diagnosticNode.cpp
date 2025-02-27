#include <actionlib/client/simple_action_client.h>
#include <eros/system_commandAction.h>
#include <gtest/gtest.h>
#include <ros/ros.h>

#include "DiagnosticNode.h"

using namespace eros;

std::string robot_namespace = "/test/";
std::string unittest_nodename = "diagnostic_node";
typedef actionlib::SimpleActionClient<eros::system_commandAction> CommandActionClient;
uint64_t heartbeat_count = 0;
eros::heartbeat latest_heartbeat;
void heartbeat_Callback(const eros::heartbeat& msg) {
    latest_heartbeat = msg;
    heartbeat_count++;
}
TEST(DiagnosticNode, TestBasics) {
    ros::NodeHandle nh("~");
    Logger* logger = new Logger("DEBUG", "test_diagnosticNode");

    logger->enable_ROS_logger();
    std::string heartbeat_topic = robot_namespace + unittest_nodename + "/heartbeat";
    ros::Subscriber sub = nh.subscribe(heartbeat_topic, 100, &heartbeat_Callback);
    sleep(5.0);
    EXPECT_NE(ros::topic::waitForMessage<eros::heartbeat>(heartbeat_topic, ros::Duration(10)),
              nullptr);
    EXPECT_EQ(1, sub.getNumPublishers());
    sleep(1.0);  // Wait for DiagnosticNode to Start.
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
        EXPECT_GT(command_pub.getNumSubscribers(), 0);
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
        sleep(5);
        EXPECT_EQ((uint8_t)Node::State::RUNNING,
                  latest_heartbeat.NodeState);  // Node should automatically change state to Running
    }

    logger->log_notice("Checking Diagnostics Service...");
    {
        std::string diagnostic_srv_topic = "/test/srv_system_diagnostics";
        ros::ServiceClient client =
            nh.serviceClient<eros::srv_get_diagnostics>(diagnostic_srv_topic);
        eros::srv_get_diagnostics srv_get_diagnostics;
        EXPECT_TRUE(client.call(srv_get_diagnostics));
    }
    logger->log_warn("Checking unsupported diagnostics service...");
    {
        std::string diagnostic_srv_topic = "/test/srv_system_diagnostics";
        ros::ServiceClient client =
            nh.serviceClient<eros::srv_get_diagnostics>(diagnostic_srv_topic);
        eros::srv_get_diagnostics srv_get_diagnostics;
        srv_get_diagnostics.request.MinLevel = (uint8_t)Level::Type::INFO;
        EXPECT_FALSE(client.call(srv_get_diagnostics));
    }

    delete logger;
}
int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "test_diagnosticNode");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    int ret = RUN_ALL_TESTS();
    spinner.stop();
    ros::shutdown();
    return ret;
}
