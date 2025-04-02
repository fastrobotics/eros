/*! \file test_armedstate_manager.cpp
 */
#include <eros/ArmedStateManager.h>
#include <eros/Logger.h>
#include <gtest/gtest.h>
#include <stdio.h>
using namespace eros;
TEST(BasicTest, TestOperation_HappyFlow) {
    std::string node_name = "UnitTestArmedStateManager";
    std::string device_name = "UnitTest";
    std::vector<std::string> ready_to_arm_list;
    ready_to_arm_list.push_back("Signal1");
    Logger* logger = new Logger("INFO", node_name);
    ArmedStateManager SUT(device_name,
                          node_name,
                          System::MainSystem::SIMROVER,
                          System::SubSystem::ROBOT_CONTROLLER,
                          logger,
                          ready_to_arm_list);
    EXPECT_EQ(SUT.get_armed_state().state, ArmDisarm::Type::DISARMED_CANNOTARM);
    auto current_diag = SUT.get_current_diagnostic();
    logger->log_diagnostic(current_diag);
    auto cannot_arm_reasons = SUT.get_cannotarm_reasons();
    ASSERT_TRUE(cannot_arm_reasons.size() > 0);
    for (auto cannot_arm_reason : cannot_arm_reasons) { logger->log_warn(cannot_arm_reason); }

    double current_time = 0.0;
    double delta_time = 0.1;
    EXPECT_EQ(current_diag.level, Level::Type::WARN);
    EXPECT_EQ(current_diag.message, eros_diagnostic::Message::INITIALIZING);
    current_diag = SUT.update(current_time += delta_time);

    EXPECT_EQ(current_diag.level, Level::Type::INFO);
    EXPECT_EQ(current_diag.message, eros_diagnostic::Message::NOERROR);

    EXPECT_TRUE(SUT.new_ready_to_arm_msg("Signal1", true));
    current_diag = SUT.update(current_time += delta_time);

    EXPECT_EQ(SUT.get_armed_state().state, ArmDisarm::Type::DISARMED);
    ASSERT_TRUE(SUT.get_cannotarm_reasons().size() == 0);

    eros::command cmd;
    cmd.Command = (uint16_t)Command::Type::ARM;
    current_diag = SUT.new_command(cmd);
    EXPECT_EQ(current_diag.level, Level::Type::INFO);
    EXPECT_EQ(current_diag.message, eros_diagnostic::Message::NOERROR);

    EXPECT_EQ(SUT.get_armed_state().state, ArmDisarm::Type::ARMING);
    double time_to_run = current_time + ArmedStateManager::ARMING_TIME_SEC + delta_time;
    while (current_time < time_to_run) {
        EXPECT_TRUE(SUT.new_ready_to_arm_msg("Signal1", true));
        current_diag = SUT.update(current_time);
        EXPECT_EQ(current_diag.level, Level::Type::INFO);
        EXPECT_EQ(current_diag.message, eros_diagnostic::Message::NOERROR);
        current_time += delta_time;
    }
    EXPECT_EQ(SUT.get_armed_state().state, ArmDisarm::Type::ARMED);

    cmd.Command = (uint16_t)Command::Type::DISARM;
    current_diag = SUT.new_command(cmd);
    EXPECT_EQ(current_diag.level, Level::Type::INFO);
    EXPECT_EQ(current_diag.message, eros_diagnostic::Message::NOERROR);

    EXPECT_EQ(SUT.get_armed_state().state, ArmDisarm::Type::DISARMING);

    time_to_run = current_time + ArmedStateManager::DISARMING_TIME_SEC + delta_time;
    while (current_time < time_to_run) {
        EXPECT_TRUE(SUT.new_ready_to_arm_msg("Signal1", true));
        current_diag = SUT.update(current_time);
        EXPECT_EQ(current_diag.level, Level::Type::INFO);
        EXPECT_EQ(current_diag.message, eros_diagnostic::Message::NOERROR);
        current_time += delta_time;
    }

    EXPECT_EQ(SUT.get_armed_state().state, ArmDisarm::Type::DISARMED);

    logger->log_info(SUT.pretty());
    delete logger;
}

TEST(BasicTest, TestOperation_MultipleArmSignals) {
    std::string node_name = "UnitTestArmedStateManager";
    std::string device_name = "UnitTest";
    std::vector<std::string> ready_to_arm_list;
    ready_to_arm_list.push_back("Signal1");
    ready_to_arm_list.push_back("Signal2");
    ready_to_arm_list.push_back("Signal3");
    Logger* logger = new Logger("INFO", node_name);
    ArmedStateManager SUT(device_name,
                          node_name,
                          System::MainSystem::SIMROVER,
                          System::SubSystem::ROBOT_CONTROLLER,
                          logger,
                          ready_to_arm_list);
    EXPECT_EQ(SUT.get_armed_state().state, ArmDisarm::Type::DISARMED_CANNOTARM);
    auto current_diag = SUT.get_current_diagnostic();

    double current_time = 0.0;
    double delta_time = 0.1;
    EXPECT_EQ(current_diag.level, Level::Type::WARN);
    EXPECT_EQ(current_diag.message, eros_diagnostic::Message::INITIALIZING);
    current_diag = SUT.update(current_time += delta_time);

    EXPECT_EQ(current_diag.level, Level::Type::INFO);
    EXPECT_EQ(current_diag.message, eros_diagnostic::Message::NOERROR);
    current_diag = SUT.update(current_time += delta_time);
    EXPECT_EQ(SUT.get_armed_state().state, ArmDisarm::Type::DISARMED_CANNOTARM);

    auto cannot_arm_reasons = SUT.get_cannotarm_reasons();
    ASSERT_TRUE(cannot_arm_reasons.size() > 0);
    for (auto cannot_arm_reason : cannot_arm_reasons) { logger->log_warn(cannot_arm_reason); }

    EXPECT_TRUE(SUT.new_ready_to_arm_msg("Signal1", true));
    current_diag = SUT.update(current_time += delta_time);

    EXPECT_FALSE(SUT.new_ready_to_arm_msg("A Signal that will never exist", true));

    EXPECT_EQ(SUT.get_armed_state().state, ArmDisarm::Type::DISARMED_CANNOTARM);

    EXPECT_TRUE(SUT.new_ready_to_arm_msg("Signal2", false));
    current_diag = SUT.update(current_time += delta_time);
    EXPECT_EQ(SUT.get_armed_state().state, ArmDisarm::Type::DISARMED_CANNOTARM);

    EXPECT_TRUE(SUT.new_ready_to_arm_msg("Signal3", false));
    current_diag = SUT.update(current_time += delta_time);
    EXPECT_EQ(SUT.get_armed_state().state, ArmDisarm::Type::DISARMED_CANNOTARM);

    eros::command cmd;
    cmd.Command = (uint16_t)Command::Type::ARM;
    current_diag = SUT.new_command(cmd);
    EXPECT_EQ(current_diag.level, Level::Type::WARN);
    EXPECT_EQ(current_diag.message, eros_diagnostic::Message::DROPPING_PACKETS);
    logger->log_diagnostic(current_diag);

    EXPECT_TRUE(SUT.new_ready_to_arm_msg("Signal1", false));
    current_diag = SUT.update(current_time += delta_time);
    EXPECT_EQ(SUT.get_armed_state().state, ArmDisarm::Type::DISARMED_CANNOTARM);
    cannot_arm_reasons = SUT.get_cannotarm_reasons();
    ASSERT_TRUE(cannot_arm_reasons.size() == 3);
    for (auto cannot_arm_reason : cannot_arm_reasons) { logger->log_warn(cannot_arm_reason); }

    EXPECT_TRUE(SUT.new_ready_to_arm_msg("Signal1", true));
    EXPECT_TRUE(SUT.new_ready_to_arm_msg("Signal2", true));
    EXPECT_TRUE(SUT.new_ready_to_arm_msg("Signal3", true));
    current_diag = SUT.update(current_time += delta_time);
    EXPECT_EQ(SUT.get_armed_state().state, ArmDisarm::Type::DISARMED);

    current_diag =
        SUT.update(current_time += ArmedStateManager::ARMED_SIGNAL_TIMEOUT_SEC + delta_time);
    EXPECT_EQ(SUT.get_armed_state().state, ArmDisarm::Type::DISARMED_CANNOTARM);
    cannot_arm_reasons = SUT.get_cannotarm_reasons();
    ASSERT_TRUE(cannot_arm_reasons.size() > 0);
    for (auto cannot_arm_reason : cannot_arm_reasons) { logger->log_warn(cannot_arm_reason); }

    EXPECT_TRUE(SUT.new_ready_to_arm_msg("Signal1", true));
    current_diag = SUT.update(current_time += delta_time);
    logger->log_info(SUT.pretty());
    EXPECT_EQ(SUT.get_armed_state().state, ArmDisarm::Type::DISARMED_CANNOTARM);

    EXPECT_TRUE(SUT.new_ready_to_arm_msg("Signal2", true));
    current_diag = SUT.update(current_time += delta_time);
    logger->log_info(SUT.pretty());
    EXPECT_EQ(SUT.get_armed_state().state, ArmDisarm::Type::DISARMED_CANNOTARM);

    EXPECT_TRUE(SUT.new_ready_to_arm_msg("Signal3", true));
    current_diag = SUT.update(current_time += delta_time);
    ASSERT_TRUE(SUT.get_cannotarm_reasons().size() == 0);
    logger->log_info(SUT.pretty());
    EXPECT_EQ(SUT.get_armed_state().state, ArmDisarm::Type::DISARMED);

    cmd.Command = (uint16_t)Command::Type::ARM;
    current_diag = SUT.new_command(cmd);
    EXPECT_EQ(current_diag.level, Level::Type::INFO);
    EXPECT_EQ(current_diag.message, eros_diagnostic::Message::NOERROR);

    EXPECT_EQ(SUT.get_armed_state().state, ArmDisarm::Type::ARMING);

    double time_to_run = current_time + ArmedStateManager::ARMING_TIME_SEC + delta_time;
    while (current_time < time_to_run) {
        EXPECT_TRUE(SUT.new_ready_to_arm_msg("Signal1", true));
        EXPECT_TRUE(SUT.new_ready_to_arm_msg("Signal2", true));
        EXPECT_TRUE(SUT.new_ready_to_arm_msg("Signal3", true));
        current_diag = SUT.update(current_time);
        logger->log_info(SUT.pretty());
        EXPECT_EQ(current_diag.level, Level::Type::INFO);
        EXPECT_EQ(current_diag.message, eros_diagnostic::Message::NOERROR);
        current_time += delta_time;
    }
    EXPECT_EQ(SUT.get_armed_state().state, ArmDisarm::Type::ARMED);

    current_diag =
        SUT.update(current_time += ArmedStateManager::ARMED_SIGNAL_TIMEOUT_SEC + delta_time);
    EXPECT_EQ(SUT.get_armed_state().state, ArmDisarm::Type::DISARMED_CANNOTARM);
    EXPECT_TRUE(SUT.new_ready_to_arm_msg("Signal1", true));
    EXPECT_TRUE(SUT.new_ready_to_arm_msg("Signal2", true));
    EXPECT_TRUE(SUT.new_ready_to_arm_msg("Signal3", true));
    current_diag = SUT.update(current_time += delta_time);
    EXPECT_EQ(SUT.get_armed_state().state, ArmDisarm::Type::DISARMED);

    logger->log_info(SUT.pretty());

    delete logger;
}

TEST(BasicTest, TestOperation_Reset) {
    std::string node_name = "UnitTestArmedStateManager";
    std::string device_name = "UnitTest";
    std::vector<std::string> ready_to_arm_list;
    ready_to_arm_list.push_back("Signal1");
    ready_to_arm_list.push_back("Signal2");
    ready_to_arm_list.push_back("Signal3");
    Logger* logger = new Logger("INFO", node_name);
    ArmedStateManager SUT(device_name,
                          node_name,
                          System::MainSystem::SIMROVER,
                          System::SubSystem::ROBOT_CONTROLLER,
                          logger,
                          ready_to_arm_list);
    EXPECT_EQ(SUT.get_armed_state().state, ArmDisarm::Type::DISARMED_CANNOTARM);
    auto current_diag = SUT.get_current_diagnostic();

    double current_time = 0.0;
    double delta_time = 0.1;
    EXPECT_EQ(current_diag.level, Level::Type::WARN);
    EXPECT_EQ(current_diag.message, eros_diagnostic::Message::INITIALIZING);
    current_diag = SUT.update(current_time += delta_time);

    EXPECT_EQ(current_diag.level, Level::Type::INFO);
    EXPECT_EQ(current_diag.message, eros_diagnostic::Message::NOERROR);

    EXPECT_TRUE(SUT.new_ready_to_arm_msg("Signal1", true));
    current_diag = SUT.update(current_time += delta_time);

    EXPECT_FALSE(SUT.new_ready_to_arm_msg("A Signal that will never exist", true));

    EXPECT_EQ(SUT.get_armed_state().state, ArmDisarm::Type::DISARMED_CANNOTARM);

    EXPECT_TRUE(SUT.new_ready_to_arm_msg("Signal2", true));
    current_diag = SUT.update(current_time += delta_time);
    EXPECT_EQ(SUT.get_armed_state().state, ArmDisarm::Type::DISARMED_CANNOTARM);

    EXPECT_TRUE(SUT.new_ready_to_arm_msg("Signal3", true));
    current_diag = SUT.update(current_time += delta_time);
    EXPECT_EQ(SUT.get_armed_state().state, ArmDisarm::Type::DISARMED);

    EXPECT_TRUE(SUT.new_ready_to_arm_msg("Signal1", true));
    current_diag = SUT.update(current_time += delta_time);
    EXPECT_EQ(SUT.get_armed_state().state, ArmDisarm::Type::DISARMED);

    eros::command cmd;
    cmd.Command = (uint16_t)Command::Type::ARM;
    current_diag = SUT.new_command(cmd);
    EXPECT_EQ(current_diag.level, Level::Type::INFO);
    EXPECT_EQ(current_diag.message, eros_diagnostic::Message::NOERROR);

    EXPECT_EQ(SUT.get_armed_state().state, ArmDisarm::Type::ARMING);

    double time_to_run = current_time + ArmedStateManager::ARMING_TIME_SEC + delta_time;
    while (current_time < time_to_run) {
        EXPECT_TRUE(SUT.new_ready_to_arm_msg("Signal1", true));
        EXPECT_TRUE(SUT.new_ready_to_arm_msg("Signal2", true));
        EXPECT_TRUE(SUT.new_ready_to_arm_msg("Signal3", true));
        current_diag = SUT.update(current_time);
        EXPECT_EQ(current_diag.level, Level::Type::INFO);
        EXPECT_EQ(current_diag.message, eros_diagnostic::Message::NOERROR);
        current_time += delta_time;
    }
    EXPECT_EQ(SUT.get_armed_state().state, ArmDisarm::Type::ARMED);
    EXPECT_TRUE(SUT.reset());

    EXPECT_EQ(SUT.get_armed_state().state, ArmDisarm::Type::DISARMED_CANNOTARM);
    EXPECT_TRUE(SUT.new_ready_to_arm_msg("Signal1", true));
    EXPECT_TRUE(SUT.new_ready_to_arm_msg("Signal2", true));
    EXPECT_TRUE(SUT.new_ready_to_arm_msg("Signal3", true));
    current_diag = SUT.update(current_time + delta_time);
    EXPECT_EQ(SUT.get_armed_state().state, ArmDisarm::Type::DISARMED);

    logger->log_info(SUT.pretty());

    delete logger;
}

TEST(BasicTest, TestOperation_FailureCases) {
    std::string node_name = "UnitTestArmedStateManager";
    std::string device_name = "UnitTest";
    std::vector<std::string> ready_to_arm_list;
    ready_to_arm_list.push_back("Signal1");
    Logger* logger = new Logger("INFO", node_name);
    ArmedStateManager SUT(device_name,
                          node_name,
                          System::MainSystem::SIMROVER,
                          System::SubSystem::ROBOT_CONTROLLER,
                          logger,
                          ready_to_arm_list);
    EXPECT_EQ(SUT.get_armed_state().state, ArmDisarm::Type::DISARMED_CANNOTARM);
    auto current_diag = SUT.get_current_diagnostic();
    double current_time = 0.0;
    double delta_time = 0.1;
    EXPECT_EQ(current_diag.level, Level::Type::WARN);
    EXPECT_EQ(current_diag.message, eros_diagnostic::Message::INITIALIZING);
    current_diag = SUT.update(current_time += delta_time);

    EXPECT_EQ(current_diag.level, Level::Type::INFO);
    EXPECT_EQ(current_diag.message, eros_diagnostic::Message::NOERROR);

    // Test Go backwards in time
    current_diag = SUT.update(0.0);

    EXPECT_EQ(current_diag.level, Level::Type::ERROR);
    EXPECT_EQ(current_diag.message, eros_diagnostic::Message::DIAGNOSTIC_FAILED);

    current_diag = SUT.update(current_time += delta_time);
    logger->log_info(SUT.pretty());
    EXPECT_EQ(current_diag.level, Level::Type::INFO);
    EXPECT_EQ(current_diag.message, eros_diagnostic::Message::NOERROR);

    // Send a Command that should never be supported
    eros::command cmd;
    cmd.Command = (uint16_t)Command::Type::END_OF_LIST;
    current_diag = SUT.new_command(cmd);
    EXPECT_EQ(current_diag.level, Level::Type::WARN);
    EXPECT_EQ(current_diag.message, eros_diagnostic::Message::DROPPING_PACKETS);

    delete logger;
}

TEST(BasicTest, TestOperation_NoSignals) {
    std::string node_name = "UnitTestArmedStateManager";
    std::string device_name = "UnitTest";
    std::vector<std::string> ready_to_arm_list;
    Logger* logger = new Logger("INFO", node_name);
    ArmedStateManager SUT(device_name,
                          node_name,
                          System::MainSystem::SIMROVER,
                          System::SubSystem::ROBOT_CONTROLLER,
                          logger,
                          ready_to_arm_list);
    EXPECT_EQ(SUT.get_armed_state().state, ArmDisarm::Type::DISARMED_CANNOTARM);
    auto current_diag = SUT.get_current_diagnostic();
    double current_time = 0.0;
    double delta_time = 0.1;
    EXPECT_EQ(current_diag.level, Level::Type::WARN);
    EXPECT_EQ(current_diag.message, eros_diagnostic::Message::INITIALIZING);
    current_diag = SUT.update(current_time += delta_time);

    EXPECT_EQ(current_diag.level, Level::Type::INFO);
    EXPECT_EQ(current_diag.message, eros_diagnostic::Message::NOERROR);
    EXPECT_EQ(SUT.get_armed_state().state, ArmDisarm::Type::DISARMED_CANNOTARM);
    auto cannot_arm_reasons = SUT.get_cannotarm_reasons();
    ASSERT_TRUE(cannot_arm_reasons.size() > 0);
    for (auto cannot_arm_reason : cannot_arm_reasons) { logger->log_warn(cannot_arm_reason); }

    delete logger;
}
int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
