/*! \file test_SafetyNodeProcess.cpp
 */
#include <gtest/gtest.h>
#include <stdio.h>

#include "SafetyNodeProcess.h"
using namespace eros;
using namespace eros_nodes::Infrastructure;
class SafetyNodeProcessTester : public SafetyNodeProcess
{
   public:
    SafetyNodeProcessTester() {
    }
    ~SafetyNodeProcessTester() {
    }
};
TEST(BasicTest, TestOperation) {
    Logger* logger = new Logger("DEBUG", "UnitTestSafetyNodeProcess");
    SafetyNodeProcessTester* tester = new SafetyNodeProcessTester;
    tester->initialize("UnitTestSafetyNodeProcess",
                       "UnitTestSafetyNodeProcess",
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
    std::vector<std::string> topics;
    topics.push_back("Topic1");
    topics.push_back("Topic2");
    topics.push_back("Topic3");
    EXPECT_TRUE(tester->set_ready_to_arm_signals(topics));

    eros_diagnostic::Diagnostic diag = tester->finish_initialization();
    EXPECT_TRUE(diag.level <= Level::Type::NOTICE);
    EXPECT_GT(tester->get_supported_commands().size(), 0);

    EXPECT_TRUE(tester->get_armed_state().armed_state ==
                (uint8_t)ArmDisarm::Type::DISARMED_CANNOTARM);
    double current_time = 0.0;
    double delta_time = 0.1;
    tester->update(delta_time, current_time += delta_time);
    {
        std::vector<std::string> cannotarm_reasons = tester->get_cannotarm_reasons();
        EXPECT_TRUE(cannotarm_reasons.size() > 0);
        for (auto reason : cannotarm_reasons) { logger->log_warn(reason); }
    }
    // Change another topic to not ready to arm
    {
        eros::ready_to_arm arm;
        arm.ready_to_arm = true;
        EXPECT_TRUE(tester->new_message_readytoarm("Topic1", arm));
    }
    {
        eros::ready_to_arm arm;
        arm.ready_to_arm = true;
        EXPECT_TRUE(tester->new_message_readytoarm("Topic2", arm));
    }
    {
        eros::ready_to_arm arm;
        arm.ready_to_arm = true;
        EXPECT_TRUE(tester->new_message_readytoarm("Topic3", arm));
    }
    tester->update(delta_time, current_time += delta_time);
    EXPECT_TRUE(tester->get_cannotarm_reasons().size() == 0);

    EXPECT_TRUE(tester->get_armed_state().armed_state == (uint8_t)ArmDisarm::Type::DISARMED);
    {
        eros::ready_to_arm arm;
        arm.ready_to_arm = false;
        EXPECT_TRUE(tester->new_message_readytoarm("Topic2", arm));
    }
    tester->update(delta_time, current_time += delta_time);
    {
        std::vector<std::string> cannotarm_reasons = tester->get_cannotarm_reasons();
        EXPECT_TRUE(cannotarm_reasons.size() == 1);
        for (auto reason : cannotarm_reasons) { logger->log_warn(reason); }
    }
    EXPECT_TRUE(tester->get_armed_state().armed_state ==
                (uint8_t)ArmDisarm::Type::DISARMED_CANNOTARM);

    EXPECT_TRUE(tester->new_message_readytoarm("Topic2", true));
    tester->update(delta_time, current_time += delta_time);
    EXPECT_TRUE(tester->get_cannotarm_reasons().size() == 0);
    EXPECT_TRUE(tester->get_armed_state().armed_state == (uint8_t)ArmDisarm::Type::DISARMED);
    {  // Arm System
        eros::command command;
        command.Command = (uint16_t)Command::Type::ARM;
        std::vector<eros_diagnostic::Diagnostic> diag_list = tester->new_commandmsg(command);
        EXPECT_TRUE(diag_list.size() > 0);
        for (auto diag : diag_list) {
            logger->log_diagnostic(diag);
            EXPECT_TRUE(diag.level <= Level::Type::NOTICE);
        }
        EXPECT_TRUE(tester->get_armed_state().armed_state == (uint8_t)ArmDisarm::Type::ARMING);
        double time_to_run = current_time + ArmedStateManager::ARMING_TIME_SEC + delta_time;
        while (current_time < time_to_run) {
            EXPECT_TRUE(tester->new_message_readytoarm("Topic1", true));
            EXPECT_TRUE(tester->new_message_readytoarm("Topic2", true));
            EXPECT_TRUE(tester->new_message_readytoarm("Topic3", true));
            tester->update(delta_time, current_time);
            current_time += delta_time;
        }
        logger->log_info(tester->pretty());
        EXPECT_TRUE(tester->get_armed_state().armed_state == (uint8_t)ArmDisarm::Type::ARMED);
    }
    // Change a Topic to Not Read to Arm
    EXPECT_TRUE(tester->new_message_readytoarm("Topic2", false));
    tester->update(delta_time, current_time += delta_time);
    EXPECT_TRUE(tester->get_armed_state().armed_state ==
                (uint8_t)ArmDisarm::Type::DISARMED_CANNOTARM);
    EXPECT_TRUE(tester->get_cannotarm_reasons().size() == 1);
    // Change another topic to not ready to arm
    {
        eros::ready_to_arm arm;
        arm.ready_to_arm = false;
        EXPECT_TRUE(tester->new_message_readytoarm("Topic3", arm));
    }
    tester->update(delta_time, current_time += delta_time);
    EXPECT_TRUE(tester->get_armed_state().armed_state ==
                (uint8_t)ArmDisarm::Type::DISARMED_CANNOTARM);
    EXPECT_TRUE(tester->get_cannotarm_reasons().size() == 2);
    // Change both back to ready to arm, state should be disarmed
    {
        eros::ready_to_arm arm;
        arm.ready_to_arm = true;
        EXPECT_TRUE(tester->new_message_readytoarm("Topic3", arm));
    }
    tester->update(delta_time, current_time += delta_time);
    EXPECT_TRUE(tester->get_armed_state().armed_state ==
                (uint8_t)ArmDisarm::Type::DISARMED_CANNOTARM);
    EXPECT_TRUE(tester->get_cannotarm_reasons().size() == 1);
    EXPECT_TRUE(tester->new_message_readytoarm("Topic2", true));
    tester->update(delta_time, current_time += delta_time);
    EXPECT_TRUE(tester->get_armed_state().armed_state == (uint8_t)ArmDisarm::Type::DISARMED);
    EXPECT_TRUE(tester->get_cannotarm_reasons().size() == 0);
    // Arm Again
    {  // Arm System
        eros::command command;
        command.Command = (uint16_t)Command::Type::ARM;
        std::vector<eros_diagnostic::Diagnostic> diag_list = tester->new_commandmsg(command);
        EXPECT_TRUE(diag_list.size() > 0);
        for (auto diag : diag_list) {
            logger->log_diagnostic(diag);
            EXPECT_TRUE(diag.level <= Level::Type::NOTICE);
        }
        EXPECT_TRUE(tester->get_armed_state().armed_state == (uint8_t)ArmDisarm::Type::ARMING);
        double time_to_run = current_time + ArmedStateManager::ARMING_TIME_SEC + delta_time;
        while (current_time < time_to_run) {
            EXPECT_TRUE(tester->new_message_readytoarm("Topic1", true));
            EXPECT_TRUE(tester->new_message_readytoarm("Topic2", true));
            EXPECT_TRUE(tester->new_message_readytoarm("Topic3", true));
            tester->update(delta_time, current_time);
            current_time += delta_time;
        }
        EXPECT_TRUE(tester->get_armed_state().armed_state == (uint8_t)ArmDisarm::Type::ARMED);
    }
    // Disarm Normally via command

    {  // Disarm System
        logger->log_info(tester->pretty());
        eros::command command;
        command.Command = (uint16_t)Command::Type::DISARM;
        std::vector<eros_diagnostic::Diagnostic> diag_list = tester->new_commandmsg(command);
        EXPECT_TRUE(diag_list.size() > 0);
        for (auto diag : diag_list) {
            logger->log_diagnostic(diag);
            EXPECT_TRUE(diag.level <= Level::Type::NOTICE);
        }
        EXPECT_TRUE(tester->get_armed_state().armed_state == (uint8_t)ArmDisarm::Type::DISARMING);
        double time_to_run = current_time + ArmedStateManager::DISARMING_TIME_SEC + delta_time;
        while (current_time < time_to_run) {
            EXPECT_TRUE(tester->new_message_readytoarm("Topic1", true));
            EXPECT_TRUE(tester->new_message_readytoarm("Topic2", true));
            EXPECT_TRUE(tester->new_message_readytoarm("Topic3", true));
            tester->update(delta_time, current_time);
            current_time += delta_time;
        }
        EXPECT_TRUE(tester->get_armed_state().armed_state == (uint8_t)ArmDisarm::Type::DISARMED);
    }

    delete logger;
    delete tester;
}
TEST(TestCommands, TestAllCommands) {
    Logger* logger = new Logger("DEBUG", "UnitTestSafetyNodeProcess");
    SafetyNodeProcessTester* tester = new SafetyNodeProcessTester;
    tester->initialize("UnitTestSafetyNodeProcess",
                       "UnitTestSafetyNodeProcess",
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
    for (uint8_t i = (uint16_t)Command::Type::UNKNOWN; i < (uint16_t)Command::Type::END_OF_LIST;
         ++i) {
        if (((Command::Type)i == Command::Type::ARM) ||
            ((Command::Type)i == Command::Type::DISARM)) {  // Don't test this, other tests do this.
            continue;
        }
        eros::command new_cmd;
        new_cmd.Command = i;
        std::vector<eros_diagnostic::Diagnostic> diag_list = tester->new_commandmsg(new_cmd);
        EXPECT_GT(diag_list.size(), 0);
        for (auto diag : diag_list) { EXPECT_TRUE(diag.level < Level::Type::WARN); }
    }

    delete logger;
}
int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
