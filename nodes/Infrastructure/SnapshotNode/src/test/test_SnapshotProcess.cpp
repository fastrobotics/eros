/*! \file test_SnapshotProcess.cpp
 */
#include <gtest/gtest.h>
#include <stdio.h>

#include "SnapshotProcess.h"
using namespace eros;
using namespace eros_nodes::Infrastructure;
class SnapshotProcessTester : public SnapshotProcess
{
   public:
    SnapshotProcessTester() {
    }
    ~SnapshotProcessTester() {
    }
};
TEST(BasicTest, TestOperation_Slave) {
    Logger* logger = new Logger("DEBUG", "UnitTestSnapshotProcess");
    SnapshotProcessTester* tester = new SnapshotProcessTester;
    eros_diagnostic::Diagnostic diag;
    tester->initialize("UnitTestSnapshotProcess",
                       "UnitTestSnapshotProcess",
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
    tester->enable_diagnostics(diagnostic_types);
    tester->set_mode(SnapshotProcess::Mode::SLAVE);
    tester->set_architecture(Architecture::Type::X86_64);
    diag = tester->finish_initialization();
    EXPECT_TRUE(diag.level <= Level::Type::NOTICE);
    EXPECT_GT(tester->get_supported_commands().size(), 0);
    EXPECT_TRUE(tester->get_logger()->log_warn("A Log to Write") ==
                Logger::LoggerStatus::LOG_WRITTEN);

    diag = tester->load_config("~/catkin_ws/src/config/Some file that doesn't exist.xml", {});
    EXPECT_TRUE(diag.level >= Level::Type::ERROR);

    diag = tester->load_config(std::string(TESTDATA_DIR) + "/config/SnapshotConfig.xml", {});
    logger->log_diagnostic(diag);
    EXPECT_TRUE(diag.level <= Level::Type::NOTICE);

    logger->log_notice("\n" + tester->pretty());

    diag = tester->finish_initialization();

    EXPECT_TRUE(diag.level <= Level::Type::NOTICE);

    eros::command command;
    command.Command = (uint16_t)Command::Type::GENERATE_SNAPSHOT;
    command.Option1 = (uint16_t)Command::GenerateSnapshot_Option1::RUN_SLAVE;
    std::vector<eros_diagnostic::Diagnostic> diag_list = tester->new_commandmsg(command);
    EXPECT_TRUE(tester->get_devicesnapshot_state() == eros::SnapshotState::STARTED);

    diag_list = tester->createnew_snapshot();
    EXPECT_TRUE(tester->get_devicesnapshot_state() == eros::SnapshotState::COMPLETE);
    tester->update(tester->HOLDCOMPLETE_TIME + 0.1, tester->HOLDCOMPLETE_TIME + 0.1);
    EXPECT_TRUE(tester->get_devicesnapshot_state() == eros::SnapshotState::NOTRUNNING);
    delete logger;
    delete tester;
}
TEST(TestCommands, TestAllCommands) {
    Logger* logger = new Logger("DEBUG", "UnitTestSnapshotProcess");
    SnapshotProcessTester* tester = new SnapshotProcessTester;
    eros_diagnostic::Diagnostic diag;
    tester->initialize("UnitTestSnapshotProcess",
                       "UnitTestSnapshotProcess",
                       "MyHost",
                       System::MainSystem::SIMROVER,
                       System::SubSystem::ENTIRE_SYSTEM,
                       System::Component::ENTIRE_SUBSYSTEM,
                       logger);
    std::vector<eros_diagnostic::DiagnosticType> diagnostic_types;
    diagnostic_types.push_back(eros_diagnostic::DiagnosticType::SOFTWARE);
    diagnostic_types.push_back(eros_diagnostic::DiagnosticType::DATA_STORAGE);
    tester->enable_diagnostics(diagnostic_types);
    tester->set_mode(SnapshotProcess::Mode::SLAVE);
    tester->set_architecture(Architecture::Type::X86_64);
    diag = tester->finish_initialization();
    EXPECT_TRUE(diag.level <= Level::Type::NOTICE);
    EXPECT_GT(tester->get_supported_commands().size(), 0);
    for (uint8_t i = (uint16_t)Command::Type::UNKNOWN; i < (uint16_t)Command::Type::END_OF_LIST;
         ++i) {
        if ((Command::Type)i ==
            Command::Type::GENERATE_SNAPSHOT) {  // Don't test this, other tests do this.
            continue;
        }
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
