/*! \file test_TeleopJoyNodeProcess.cpp
 */
#include <gtest/gtest.h>
#include <stdio.h>

#include "TeleopJoyNodeProcess.h"
using namespace eros;
using namespace eros_nodes::Infrastructure;
class TeleopJoyNodeProcessTester : public TeleopJoyNodeProcess
{
   public:
    TeleopJoyNodeProcessTester() {
    }
    ~TeleopJoyNodeProcessTester() {
    }
};
TEST(BasicTest, TestOperation) {
    Logger* logger = new Logger("DEBUG", "UnitTestTeleopJoyNodeProcess");
    TeleopJoyNodeProcessTester* tester = new TeleopJoyNodeProcessTester;
    tester->initialize("UnitTestTeleopJoyNodeProcess",
                       "UnitTestTeleopJoyNodeProcess",
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
    tester->enable_diagnostics(diagnostic_types);
    EXPECT_TRUE(tester->get_logger()->log_warn("A Log to Write") ==
                Logger::LoggerStatus::LOG_WRITTEN);

    delete logger;
    delete tester;
}
TEST(TestCommands, TestAllCommands) {
    Logger* logger = new Logger("DEBUG", "UnitTestTeleopJoyNodeProcess");
    TeleopJoyNodeProcessTester* tester = new TeleopJoyNodeProcessTester;
    tester->initialize("UnitTestTeleopJoyNodeProcess",
                       "UnitTestTeleopJoyNodeProcess",
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
    tester->enable_diagnostics(diagnostic_types);
    eros_diagnostic::Diagnostic diag = tester->finish_initialization();
    EXPECT_TRUE(diag.level <= Level::Type::NOTICE);
    for (uint8_t i = (uint16_t)Command::Type::UNKNOWN; i < (uint16_t)Command::Type::END_OF_LIST;
         ++i) {
        eros::command new_cmd;
        new_cmd.Command = i;
        std::vector<eros_diagnostic::Diagnostic> diag_list = tester->new_commandmsg(new_cmd);
        EXPECT_GT(diag_list.size(), 0);
        for (auto diag : diag_list) { EXPECT_TRUE(diag.level < Level::Type::WARN); }
    }

    delete tester;
    delete logger;
}
int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
