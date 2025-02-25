/*! \file test_DataLoggerProcess.cpp
 */

#include <gtest/gtest.h>
#include <stdio.h>

#include "DataLoggerProcess.h"
using namespace eros;
using namespace eros_nodes::Infrastructure;
class DataLoggerProcessTester : public DataLoggerProcess
{
   public:
    DataLoggerProcessTester() {
    }
    ~DataLoggerProcessTester() {
    }
};
TEST(BasicTest, TestOperation) {
    Logger* logger = new Logger("DEBUG", "UnitTestDataLoggerProcess");
    DataLoggerProcessTester* tester = new DataLoggerProcessTester;
    tester->initialize("UnitTestDataLoggerProcess",
                       "UnitTestDataLoggerProcess",
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

    tester->reset();

    {
        eros::command cmd;
        std::vector<eros_diagnostic::Diagnostic> diag_list = tester->new_commandmsg(cmd);
        EXPECT_EQ(diag_list.size(), 0);
    }
    {
        std::vector<eros_diagnostic::Diagnostic> diag_list = tester->check_programvariables();
        EXPECT_EQ(diag_list.size(), 0);
    }

    delete logger;
    delete tester;
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
