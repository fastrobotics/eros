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
    SUT.enable_diagnostics(diagnostic_types);
    EXPECT_TRUE(SUT.get_logger()->log_warn("A Log to Write") == Logger::LoggerStatus::LOG_WRITTEN);
    eros_diagnostic::Diagnostic diag = SUT.finish_initialization();
    EXPECT_TRUE(diag.level < Level::Type::WARN);
    EXPECT_TRUE(SUT.set_nodeHandle((&nh), "/"));

    {
        diag = SUT.update(0.0, 0.0);
        EXPECT_TRUE(diag.level < Level::Type::WARN);
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