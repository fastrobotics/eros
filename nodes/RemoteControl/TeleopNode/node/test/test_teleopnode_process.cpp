/*! \file test_teleopnode_process.cpp
 */
// clang-format off
#include <gtest/gtest.h>
#include "../TeleopNodeProcess.h"
#include <stdio.h>
// clang-format on
using namespace eros;

namespace eros_nodes::RemoteControl {}  // namespace eros_nodes::RemoteControl
using namespace eros_nodes::RemoteControl;
TEST(BasicTest, TestOperation) {
    ros::NodeHandle nh("~");
    Logger* logger = new Logger("DEBUG", "UnitTestTeleopNodeProcess");
    TeleopNodeProcess SUT;
    SUT.initialize("UnitTestTeleopNodeProcess",
                   "UnitTestTeleopNodeProcess",
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
    diagnostic_types.push_back(eros_diagnostic::DiagnosticType::REMOTE_CONTROL);
    SUT.enable_diagnostics(diagnostic_types);
    eros_diagnostic::Diagnostic diag = SUT.finish_initialization();
    EXPECT_TRUE(SUT.get_logger()->log_warn("A Log to Write") == Logger::LoggerStatus::LOG_WRITTEN);
    EXPECT_TRUE(SUT.set_nodeHandle((&nh), "/"));

    {
        diag = SUT.update(0.0, 0.0);
        EXPECT_TRUE(diag.level < Level::Type::WARN);
    }
    delete logger;
}
TEST(TestCommands, TestAllCommands) {
    ros::NodeHandle nh("~");
    Logger* logger = new Logger("DEBUG", "UnitTestTeleopNodeProcess");
    TeleopNodeProcess SUT;
    SUT.initialize("UnitTestTeleopNodeProcess",
                   "UnitTestTeleopNodeProcess",
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
    diagnostic_types.push_back(eros_diagnostic::DiagnosticType::REMOTE_CONTROL);
    SUT.enable_diagnostics(diagnostic_types);
    eros_diagnostic::Diagnostic diag = SUT.finish_initialization();
    EXPECT_TRUE(diag.level <= Level::Type::NOTICE);
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
    ros::init(argc, argv, "test_teleopnode_process");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    int ret = RUN_ALL_TESTS();
    spinner.stop();
    ros::shutdown();
    return ret;
}