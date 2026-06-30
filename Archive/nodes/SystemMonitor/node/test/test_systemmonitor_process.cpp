/*! \file test_systemmonitor_process.cpp
 */
// clang-format off
#include <gtest/gtest.h>
#include "../SystemMonitorProcess.h"
#include <stdio.h>
// clang-format on
using namespace eros;

namespace eros_nodes::SystemMonitor {}  // namespace eros_nodes::SystemMonitor
using namespace eros_nodes::SystemMonitor;
TEST(BasicTest, TestOperation) {
    ros::NodeHandle nh("~");
    Logger* logger = new Logger("DEBUG", "UnitTestSystemMonitorProcess");
    SystemMonitorProcess SUT;
    SUT.initialize("UnitTestSystemMonitorProcess",
                   "UnitTestSystemMonitorProcess",
                   "MyHost",
                   System::MainSystem::SIMROVER,
                   System::SubSystem::ENTIRE_SYSTEM,
                   System::Component::ENTIRE_SUBSYSTEM,
                   logger);
    std::vector<eros_diagnostic::DiagnosticType> diagnostic_types;
    diagnostic_types.push_back(eros_diagnostic::DiagnosticType::SOFTWARE);
    diagnostic_types.push_back(eros_diagnostic::DiagnosticType::DATA_STORAGE);
    diagnostic_types.push_back(eros_diagnostic::DiagnosticType::SYSTEM_RESOURCE);
    diagnostic_types.push_back(eros_diagnostic::DiagnosticType::COMMUNICATIONS);
    SUT.enable_diagnostics(diagnostic_types);
    EXPECT_TRUE(SUT.get_logger()->log_warn("A Log to Write") == Logger::LoggerStatus::LOG_WRITTEN);
    eros_diagnostic::Diagnostic diag = SUT.finish_initialization();
    EXPECT_TRUE(diag.level < Level::Type::WARN);
    EXPECT_TRUE(SUT.set_nodeHandle((&nh), "/"));
    // EXPECT_FALSE(SUT.initialize_windows());  // Can't initialize windows during unit tests
    // Check Monitor List
    {
        std::vector<std::string> heartbeat_list;
        heartbeat_list.push_back("/heartbeat/Topic1");
        std::vector<std::string> resource_used_list;
        resource_used_list.push_back("/resource_used/Topic1");
        std::vector<std::string> loadfactor_list;
        loadfactor_list.push_back("/loadfactor/Topic1");
        std::vector<std::string> resourceavailable_list;
        resourceavailable_list.push_back("/resource_available/Topic1");
        std::vector<std::string> new_heartbeat_topics_to_subscribe;
        std::vector<std::string> new_resourceused_topics_to_subscribe;
        std::vector<std::string> new_loadfactor_topics_to_subscribe;
        std::vector<std::string> new_resourceavailable_topics_to_subscribe;
        diag = SUT.update_monitorlist(heartbeat_list,
                                      resource_used_list,
                                      resourceavailable_list,
                                      loadfactor_list,
                                      new_heartbeat_topics_to_subscribe,
                                      new_resourceused_topics_to_subscribe,
                                      new_resourceavailable_topics_to_subscribe,
                                      new_loadfactor_topics_to_subscribe);
        EXPECT_TRUE(diag.level < Level::Type::WARN);
        EXPECT_EQ(new_heartbeat_topics_to_subscribe.size(), 1);
        EXPECT_EQ(new_resourceused_topics_to_subscribe.size(), 1);
        EXPECT_EQ(new_loadfactor_topics_to_subscribe.size(), 1);
        EXPECT_EQ(new_resourceavailable_topics_to_subscribe.size(), 1);
    }
    {
        std::vector<std::string> heartbeat_list;
        heartbeat_list.push_back("/heartbeat/Topic1");
        std::vector<std::string> resource_used_list;
        resource_used_list.push_back("/resource_used/Topic1");
        std::vector<std::string> loadfactor_list;
        loadfactor_list.push_back("/loadfactor/Topic1");
        std::vector<std::string> resourceavailable_list;
        resourceavailable_list.push_back("/resource_available/Topic1");
        std::vector<std::string> new_heartbeat_topics_to_subscribe;
        std::vector<std::string> new_resourceused_topics_to_subscribe;
        std::vector<std::string> new_loadfactor_topics_to_subscribe;
        std::vector<std::string> new_resourceavailable_topics_to_subscribe;
        diag = SUT.update_monitorlist(heartbeat_list,
                                      resource_used_list,
                                      resourceavailable_list,
                                      loadfactor_list,
                                      new_heartbeat_topics_to_subscribe,
                                      new_resourceused_topics_to_subscribe,
                                      new_resourceavailable_topics_to_subscribe,
                                      new_loadfactor_topics_to_subscribe);
        EXPECT_TRUE(diag.level < Level::Type::WARN);
        EXPECT_EQ(new_heartbeat_topics_to_subscribe.size(), 0);
        EXPECT_EQ(new_resourceused_topics_to_subscribe.size(), 0);
        EXPECT_EQ(new_loadfactor_topics_to_subscribe.size(), 0);
        EXPECT_EQ(new_resourceavailable_topics_to_subscribe.size(), 0);
    }
    {
        diag = SUT.update(0.0, 0.0);
        EXPECT_TRUE(diag.level < Level::Type::WARN);
    }
    delete logger;
}
TEST(TestCommands, TestAllCommands) {
    ros::NodeHandle nh("~");
    Logger* logger = new Logger("DEBUG", "UnitTestSystemMonitorProcess");
    SystemMonitorProcess SUT;
    SUT.initialize("UnitTestSystemMonitorProcess",
                   "UnitTestSystemMonitorProcess",
                   "MyHost",
                   System::MainSystem::SIMROVER,
                   System::SubSystem::ENTIRE_SYSTEM,
                   System::Component::ENTIRE_SUBSYSTEM,
                   logger);
    std::vector<eros_diagnostic::DiagnosticType> diagnostic_types;
    diagnostic_types.push_back(eros_diagnostic::DiagnosticType::SOFTWARE);
    diagnostic_types.push_back(eros_diagnostic::DiagnosticType::DATA_STORAGE);
    diagnostic_types.push_back(eros_diagnostic::DiagnosticType::SYSTEM_RESOURCE);
    diagnostic_types.push_back(eros_diagnostic::DiagnosticType::COMMUNICATIONS);
    SUT.enable_diagnostics(diagnostic_types);
    EXPECT_TRUE(SUT.get_logger()->log_warn("A Log to Write") == Logger::LoggerStatus::LOG_WRITTEN);
    eros_diagnostic::Diagnostic diag = SUT.finish_initialization();
    EXPECT_TRUE(diag.level < Level::Type::WARN);
    for (uint8_t i = (uint16_t)Command::Type::UNKNOWN; i < (uint16_t)Command::Type::END_OF_LIST;
         ++i) {
        eros::command new_cmd;
        new_cmd.Command = i;
        std::vector<eros_diagnostic::Diagnostic> diag_list = SUT.new_commandmsg(new_cmd);
        EXPECT_GT(diag_list.size(), 0);
        for (auto diag : diag_list) { EXPECT_TRUE(diag.level < Level::Type::WARN); }
    }

    delete logger;
}
int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "test_systemmonitor_process");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    int ret = RUN_ALL_TESTS();
    spinner.stop();
    ros::shutdown();
    return ret;
}