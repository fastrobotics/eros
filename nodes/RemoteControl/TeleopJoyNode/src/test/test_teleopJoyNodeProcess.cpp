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
int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
