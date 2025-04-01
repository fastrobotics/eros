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

    tester->reset();
    double cur_time = 0.0;
    double dt = 0.1;
    while (cur_time <= 20.0) {
        {
            auto diag = tester->update(dt, cur_time);
            EXPECT_TRUE(diag.level <= eros::Level::Type::NOTICE);
        }
        sensor_msgs::Joy joy_msg;
        joy_msg.axes.resize(8);
        joy_msg.buttons.resize(8);
        {
            auto diag_list = tester->new_joymsg(joy_msg);
            EXPECT_GT(diag_list.size(), 0);
            for (auto diag : diag_list) { EXPECT_TRUE(diag.level <= eros::Level::Type::NOTICE); }
        }
        cur_time += dt;
    }
    logger->log_info(tester->pretty());
    auto diag_list = tester->check_programvariables();
    EXPECT_EQ(diag_list.size(), 0);

    delete logger;
    delete tester;
}
TEST(TestCommand, TestCommandArmDisarm) {
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

    auto command = tester->get_current_command();
    EXPECT_FALSE(command.new_command);

    // Joystick Message
    sensor_msgs::Joy joy_msg;
    joy_msg.axes.resize(8);
    joy_msg.buttons.resize(8);
    joy_msg.buttons[TeleopJoyNodeProcess::JOY_BUTTON_COMMAND_ARMDISARM] = 1;

    logger->log_warn(
        "The following test should print a warning about not allowing an Arm/Disarm Command.");
    auto diag_list = tester->new_joymsg(joy_msg);
    EXPECT_GT(diag_list.size(), 0);
    bool passed_remote_control_diag_check = false;
    for (auto diag : diag_list) {
        if (diag.type == eros_diagnostic::DiagnosticType::REMOTE_CONTROL) {
            EXPECT_TRUE(diag.level >= eros::Level::Type::WARN);
            passed_remote_control_diag_check = true;
        }
        else {
            EXPECT_TRUE(diag.level <= eros::Level::Type::NOTICE);
        }
    }
    EXPECT_TRUE(passed_remote_control_diag_check);

    eros::ArmDisarm::State armed_state;
    armed_state.state = eros::ArmDisarm::Type::DISARMED;
    tester->update_armedstate(armed_state);

    diag_list = tester->new_joymsg(joy_msg);
    EXPECT_GT(diag_list.size(), 0);
    for (auto diag : diag_list) { EXPECT_TRUE(diag.level <= eros::Level::Type::NOTICE); }

    command = tester->get_current_command();
    EXPECT_TRUE(command.new_command);
    EXPECT_EQ(command.command.Command, (uint16_t)eros::Command::Type::ARM);
    command = tester->get_current_command();
    EXPECT_FALSE(command.new_command);

    armed_state.state = eros::ArmDisarm::Type::ARMED;
    tester->update_armedstate(armed_state);

    diag_list = tester->new_joymsg(joy_msg);
    EXPECT_GT(diag_list.size(), 0);
    for (auto diag : diag_list) { EXPECT_TRUE(diag.level <= eros::Level::Type::NOTICE); }

    command = tester->get_current_command();
    EXPECT_TRUE(command.new_command);
    EXPECT_EQ(command.command.Command, (uint16_t)eros::Command::Type::DISARM);
    command = tester->get_current_command();
    EXPECT_FALSE(command.new_command);

    delete tester;
    delete logger;
}
TEST(TestCommand, TestCommandSnapshot) {
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

    auto command = tester->get_current_command();
    EXPECT_FALSE(command.new_command);

    // Joystick Message
    sensor_msgs::Joy joy_msg;
    joy_msg.axes.resize(8);
    joy_msg.buttons.resize(8);
    joy_msg.buttons[TeleopJoyNodeProcess::JOY_BUTTON_COMMAND_SNAPSHOT] = 1;

    auto diag_list = tester->new_joymsg(joy_msg);
    EXPECT_GT(diag_list.size(), 0);
    for (auto diag : diag_list) { EXPECT_TRUE(diag.level <= eros::Level::Type::NOTICE); }

    command = tester->get_current_command();
    EXPECT_TRUE(command.new_command);
    EXPECT_EQ(command.command.Command, (uint16_t)eros::Command::Type::GENERATE_SNAPSHOT);
    command = tester->get_current_command();
    EXPECT_FALSE(command.new_command);

    delete tester;
    delete logger;
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
    ros::Time::init();
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
