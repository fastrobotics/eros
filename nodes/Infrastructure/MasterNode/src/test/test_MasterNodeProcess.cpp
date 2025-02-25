/*! \file test_MasterNodeProcess.cpp
 */
#include <gtest/gtest.h>
#include <stdio.h>

#include "MasterNodeProcess.h"
using namespace eros;
using namespace eros_nodes::Infrastructure;
class MasterNodeProcessTester : public MasterNodeProcess
{
   public:
    MasterNodeProcessTester() {
    }
    ~MasterNodeProcessTester() {
    }
};
TEST(BasicTest, TestOperation) {
    Logger* logger = new Logger("DEBUG", "UnitTestMasterNodeProcess");
    MasterNodeProcessTester* tester = new MasterNodeProcessTester;
    tester->initialize("UnitTestMasterNodeProcess",
                       "UnitTestMasterNodeProcess",
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
    EXPECT_TRUE(tester->get_logger()->log_warn("A Log to Write") ==
                Logger::LoggerStatus::LOG_WRITTEN);

    eros_diagnostic::Diagnostic diag = tester->finish_initialization();
    logger->log_diagnostic(diag);
    EXPECT_TRUE(diag.level <= Level::Type::NOTICE);

    tester->reset();

    double timeToRun = 10.0;
    double dt = 0.1;
    double timer = 0.0;
    while (timer <= timeToRun) {
        diag = tester->update(dt, timer);
        EXPECT_TRUE(diag.level <= Level::Type::NOTICE);
        timer += dt;
    }

    logger->log_warn("Testing Unsupported Command Message");
    {
        eros::command cmd;
        std::vector<eros::eros_diagnostic::Diagnostic> diag_list = tester->new_commandmsg(cmd);
        EXPECT_EQ(diag_list.size(), 0);
    }
    logger->log_warn("Testing Unsupported Program Variables Check");
    {
        std::vector<eros::eros_diagnostic::Diagnostic> diag_list = tester->check_programvariables();
        EXPECT_EQ(diag_list.size(), 0);
    }
    tester->cleanup();

    delete logger;
    delete tester;
}
int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
