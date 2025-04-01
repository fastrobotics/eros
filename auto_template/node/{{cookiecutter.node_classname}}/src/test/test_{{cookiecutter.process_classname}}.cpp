/*! \file test_{{cookiecutter.process_classname}}.cpp
 */
#include <gtest/gtest.h>
#include <stdio.h>

#include "{{cookiecutter.process_classname}}.h"
using namespace eros;
using namespace {{cookiecutter.package_name}};
class {{cookiecutter.process_classname}}Tester : public {{cookiecutter.process_classname}}
{
   public:
    {{cookiecutter.process_classname}}Tester() {
    }
    ~{{cookiecutter.process_classname}}Tester() {
    }
};
TEST(BasicTest, TestOperation) {
    Logger* logger = new Logger("DEBUG", "UnitTest{{cookiecutter.process_classname}}");
    {{cookiecutter.process_classname}}Tester* tester = new {{cookiecutter.process_classname}}Tester;
    tester->initialize("UnitTest{{cookiecutter.process_classname}}",
                       "UnitTest{{cookiecutter.process_classname}}",
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
    logger->log_diagnostic(diag);
    EXPECT_TRUE(diag.level <= Level::Type::NOTICE);
    EXPECT_TRUE(tester->get_logger()->log_warn("A Log to Write") ==
                Logger::LoggerStatus::LOG_WRITTEN);

    tester->reset();

    double timeToRun = 10.0;
    double dt = 0.1;
    double timer = 0.0;
    while (timer <= timeToRun) {
        diag = tester->update(dt, timer);
        EXPECT_TRUE(diag.level <= Level::Type::NOTICE);
        timer += dt;
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
TEST(TestCommands, TestAllCommands) {
    Logger* logger = new Logger("DEBUG", "UnitTest{{cookiecutter.process_classname}}");
    {{cookiecutter.process_classname}}Tester* tester = new {{cookiecutter.process_classname}}Tester;
    tester->initialize("UnitTest{{cookiecutter.process_classname}}",
                       "UnitTest{{cookiecutter.process_classname}}",
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
