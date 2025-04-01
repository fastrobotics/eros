/*! \file test_DiagnosticNodeProcess.cpp
 */
#include <gtest/gtest.h>
#include <stdio.h>

#include "DiagnosticNodeProcess.h"
using namespace eros;
using namespace eros_nodes::Infrastructure;
class DiagnosticNodeProcessTester : public DiagnosticNodeProcess
{
   public:
    DiagnosticNodeProcessTester() {
    }
    ~DiagnosticNodeProcessTester() {
    }
};
TEST(BasicTest, TestOperation) {
    Logger* logger = new Logger("DEBUG", "UnitTestDiagnosticNodeProcess");
    DiagnosticNodeProcessTester* tester = new DiagnosticNodeProcessTester;
    tester->initialize("UnitTestDiagnosticNodeProcess",
                       "UnitTestDiagnosticNodeProcess",
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
    eros_diagnostic::Diagnostic diag = tester->finish_initialization();
    EXPECT_TRUE(diag.level <= Level::Type::NOTICE);
    EXPECT_TRUE(tester->get_logger()->log_warn("A Log to Write") ==
                Logger::LoggerStatus::LOG_WRITTEN);
    tester->finish_initialization();

    double timeToRun = 10.0;
    double dt = 0.1;
    double timer = 0.0;
    while (timer <= timeToRun) {
        diag = tester->update(dt, timer);
        EXPECT_TRUE(diag.level <= Level::Type::NOTICE);
        timer += dt;
    }
    tester->reset();
    {
        eros_diagnostic::Diagnostic worst_diag =
            tester->get_worst_diagnostic(eros_diagnostic::DiagnosticType::REMOTE_CONTROL);
        logger->log_diagnostic(worst_diag);
    }
    EXPECT_TRUE(
        tester->get_worst_diagnostic(eros_diagnostic::DiagnosticType::REMOTE_CONTROL).level ==
        Level::Type::INFO);
    {
        eros_diagnostic::Diagnostic diag("Device1",
                                         "Node1",
                                         System::MainSystem::ROVER,
                                         System::SubSystem::ROBOT_CONTROLLER,
                                         System::Component::COMMUNICATION,
                                         eros_diagnostic::DiagnosticType::REMOTE_CONTROL,
                                         eros_diagnostic::Message::INITIALIZING,
                                         Level::Type::WARN,
                                         "Msg1");
        EXPECT_TRUE(tester->new_external_diagnostic(diag));
    }

    EXPECT_TRUE(
        tester->get_worst_diagnostic(eros_diagnostic::DiagnosticType::REMOTE_CONTROL).level ==
        Level::Type::WARN);
    {
        eros_diagnostic::Diagnostic diag("Device1",
                                         "Node1",
                                         System::MainSystem::ROVER,
                                         System::SubSystem::ROBOT_CONTROLLER,
                                         System::Component::DIAGNOSTIC,
                                         eros_diagnostic::DiagnosticType::REMOTE_CONTROL,
                                         eros_diagnostic::Message::INITIALIZING,
                                         Level::Type::NOTICE,
                                         "Msg1");
        EXPECT_TRUE(tester->new_external_diagnostic(diag));
    }
    EXPECT_TRUE(
        tester->get_worst_diagnostic(eros_diagnostic::DiagnosticType::REMOTE_CONTROL).level ==
        Level::Type::WARN);
    {
        eros_diagnostic::Diagnostic diag("Device1",
                                         "Node1",
                                         System::MainSystem::ROVER,
                                         System::SubSystem::ROBOT_CONTROLLER,
                                         System::Component::COMMUNICATION,
                                         eros_diagnostic::DiagnosticType::REMOTE_CONTROL,
                                         eros_diagnostic::Message::INITIALIZING,
                                         Level::Type::NOTICE,
                                         "Msg1");
        EXPECT_TRUE(tester->new_external_diagnostic(diag));
    }
    EXPECT_TRUE(
        tester->get_worst_diagnostic(eros_diagnostic::DiagnosticType::REMOTE_CONTROL).level ==
        Level::Type::NOTICE);
    printf("%s\n", tester->pretty().c_str());

    logger->log_warn("Testing Unsupported Program Variables Check");
    {
        std::vector<eros::eros_diagnostic::Diagnostic> diag_list = tester->check_programvariables();
        EXPECT_EQ(diag_list.size(), 0);
    }

    logger->log_notice("Check Topic Subscribe List.");
    {
        std::vector<std::string> topic_list = {"/a", "/b", "/c"};
        eros_diagnostic::Diagnostic diag = tester->update_topiclist(topic_list);
        EXPECT_TRUE(diag.level <= Level::Type::NOTICE);

        std::vector<std::string> redundant_topic_list = {"/a", "/d", "/e"};
        diag = tester->update_topiclist(redundant_topic_list);
        EXPECT_TRUE(diag.level <= Level::Type::NOTICE);
    }
    logger->log_notice("Check Diagnostic Aggregator.");
    {
        eros_diagnostic::Diagnostic badDiag("Device1",
                                            "Node1",
                                            System::MainSystem::ROVER,
                                            System::SubSystem::ROBOT_CONTROLLER,
                                            System::Component::COMMUNICATION,
                                            eros_diagnostic::DiagnosticType::REMOTE_CONTROL,
                                            eros_diagnostic::Message::DROPPING_PACKETS,
                                            Level::Type::ERROR,
                                            "Dropping Packets");
        EXPECT_TRUE(tester->new_external_diagnostic(badDiag));
        logger->log_notice(tester->pretty());
    }
    logger->log_notice("Check Failure Cases");
    {
        eros_diagnostic::Diagnostic unknownDiag;
        unknownDiag.type = eros_diagnostic::DiagnosticType::UNKNOWN;
        EXPECT_FALSE(tester->new_external_diagnostic(unknownDiag));
    }
    tester->cleanup();
    delete logger;
    delete tester;
}
TEST(TestCommands, TestAllCommands) {
    Logger* logger = new Logger("DEBUG", "UnitTestDiagnosticNodeProcess");
    DiagnosticNodeProcessTester* tester = new DiagnosticNodeProcessTester;
    tester->initialize("UnitTestDiagnosticNodeProcess",
                       "UnitTestDiagnosticNodeProcess",
                       "MyHost",
                       System::MainSystem::SIMROVER,
                       System::SubSystem::ENTIRE_SYSTEM,
                       System::Component::ENTIRE_SUBSYSTEM,
                       logger);
    std::vector<eros_diagnostic::DiagnosticType> diagnostic_types;
    diagnostic_types.push_back(eros_diagnostic::DiagnosticType::SOFTWARE);
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
