#include <actionlib/client/simple_action_client.h>
#include <eros/system_commandAction.h>
#include <gtest/gtest.h>
#include <ros/ros.h>

#include "MasterNode.h"

using namespace eros;
std::string robot_namespace = "/test/";
std::string unittest_nodename = "master_node";
typedef actionlib::SimpleActionClient<eros::system_commandAction> CommandActionClient;
uint64_t heartbeat_count = 0;
eros::heartbeat latest_heartbeat;
void heartbeat_Callback(const eros::heartbeat& msg) {
    latest_heartbeat = msg;
    heartbeat_count++;
}
uint64_t diagnostic_count = 0;
eros::diagnostic latest_diagnostic;
void diagnostic_Callback(const eros::diagnostic& msg) {
    latest_diagnostic = msg;
    diagnostic_count++;
}
TEST(MasterNode, TestMaster) {
    ros::NodeHandle nh("~");
    Logger* logger = new Logger("DEBUG", "test_masterNode");
    logger->enable_ROS_logger();

    std::string heartbeat_topic = robot_namespace + unittest_nodename + "/heartbeat";
    std::string diagnostic_topic = robot_namespace + unittest_nodename + "/diagnostic";
    ros::Subscriber heartbeat_sub = nh.subscribe(heartbeat_topic, 100, &heartbeat_Callback);
    ros::Subscriber diagnostic_sub = nh.subscribe(diagnostic_topic, 100, &diagnostic_Callback);
    sleep(5.0);
    EXPECT_NE(ros::topic::waitForMessage<eros::heartbeat>(heartbeat_topic, ros::Duration(10)),
              nullptr);
    EXPECT_EQ(1, heartbeat_sub.getNumPublishers());
    EXPECT_NE(ros::topic::waitForMessage<eros::diagnostic>(diagnostic_topic, ros::Duration(10)),
              nullptr);
    EXPECT_EQ(1, diagnostic_sub.getNumPublishers());

    ros::Publisher armedstate_pub =
        nh.advertise<eros::armed_state>(robot_namespace + "/ArmedState", 20);

    ros::Publisher modestate_pub =
        nh.advertise<eros::mode_state>(robot_namespace + "/ModeState", 20);
    sleep(1.0);  // Wait for Master Node to Start.

    EXPECT_GT(heartbeat_count, 0);
    EXPECT_GT(diagnostic_count, 0);
    EXPECT_GT(armedstate_pub.getNumSubscribers(), 0);
    EXPECT_GT(modestate_pub.getNumSubscribers(), 0);
    eros::armed_state armedState;
    armedstate_pub.publish(armedState);

    eros::mode_state modeState;
    modestate_pub.publish(modeState);
    {
        ros::ServiceClient client = nh.serviceClient<eros::srv_firmware>(
            robot_namespace + unittest_nodename + "/srv_firmware");
        eros::srv_firmware req;
        EXPECT_TRUE(client.call(req));
        EXPECT_EQ(req.response.BaseNodeName, unittest_nodename);
    }

    {
        ros::ServiceClient client = nh.serviceClient<eros::srv_logger_level>(
            robot_namespace + unittest_nodename + "/srv_loggerlevel");
        eros::srv_logger_level req;
        req.request.LoggerLevel = "UNKNOWN";
        logger->log_warn("Checking Failure Case...");
        EXPECT_FALSE(client.call(req));
        req.request.LoggerLevel = "DEBUG";
        EXPECT_TRUE(client.call(req));
        req.request.LoggerLevel = "NOTICE";
        EXPECT_TRUE(client.call(req));
    }
    {
        ros::ServiceClient client = nh.serviceClient<eros::srv_get_diagnostics>(
            robot_namespace + unittest_nodename + "/srv_diagnostics");
        eros::srv_get_diagnostics req;
        req.request.MinLevel = 0;        // All
        req.request.DiagnosticType = 0;  // All
        EXPECT_TRUE(client.call(req));
        logger->log_diagnostic(
            eros::eros_diagnostic::DiagnosticUtility::convert(req.response.worst_diag));
        EXPECT_LT(req.response.worst_diag.Level, (uint8_t)Level::Type::WARN);
        for (auto diag : req.response.diag_list) {
            EXPECT_LT(diag.Level, (uint8_t)Level::Type::WARN);
        }
    }

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
        sleep(5);
        EXPECT_EQ((uint8_t)Node::State::RUNNING,
                  latest_heartbeat.NodeState);  // Node should automatically change state to Running
    }
    sleep(1.0);

    logger->log_notice("Test Device Service...");
    {
        ros::ServiceClient client =
            nh.serviceClient<eros::srv_device>(robot_namespace + unittest_nodename + "/srv_device");
        eros::srv_device req;
        EXPECT_TRUE(client.call(req));
    }

    delete logger;
}
int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "test_masterNode");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    int ret = RUN_ALL_TESTS();
    spinner.stop();
    ros::shutdown();
    return ret;
}
