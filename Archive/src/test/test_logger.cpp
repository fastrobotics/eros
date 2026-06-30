/*! \file test_diagnostics.cpp
 */
#include <eros/Logger.h>
#include <eros_diagnostic/Diagnostic.h>
#include <gtest/gtest.h>
#include <stdio.h>
using namespace eros;
TEST(BasicTest, TestTypicalOperation) {
    eros_diagnostic::Diagnostic diag("UnitTestDevice",
                                     "UnitTestNode-Logger",
                                     System::MainSystem::SIMROVER,
                                     System::SubSystem::ENTIRE_SYSTEM,
                                     System::Component::ENTIRE_SUBSYSTEM);
    Logger* logger = new Logger("INFO", diag.node_name);
    EXPECT_TRUE(logger->is_logger_ok());
    EXPECT_TRUE(logger->enable_ROS_logger() == true);
    EXPECT_TRUE(logger->log_debug("A String that should debug") ==
                Logger::LoggerStatus::LOG_SUPPRESSED);
    EXPECT_TRUE(logger->log_info("A String that should info") == Logger::LoggerStatus::LOG_WRITTEN);
    EXPECT_TRUE(logger->log_notice("A String that should notice") ==
                Logger::LoggerStatus::LOG_WRITTEN);
    EXPECT_TRUE(logger->log_warn("A String that should warn") == Logger::LoggerStatus::LOG_WRITTEN);
    EXPECT_TRUE(logger->log_error("A String that should error") ==
                Logger::LoggerStatus::LOG_WRITTEN);
    EXPECT_TRUE(logger->log_fatal("A String that should fatal") ==
                Logger::LoggerStatus::LOG_WRITTEN);

    logger->log_warn("Checking Enable/Disable Console Print.");
    logger->disable_consoleprint();
    logger->log_warn("Text should now only be in log file, not in console.");
    logger->enable_consoleprint();
    logger->log_warn("Text should now be in console again.");
    delete logger;
}
TEST(FailureTests, FailureCases) {
    {
        Logger* logger = new Logger("INFO", "/RootNodeName");
        EXPECT_TRUE(logger->is_logger_ok());
        delete logger;
    }
    {
        Logger* logger = new Logger("DEBUG", "/ADirectoryThatShouldNeverExist", "ABadLogFile");
        EXPECT_FALSE(logger->is_logger_ok());
        Logger::LoggerStatus status = logger->log_debug("The Logger has Failed on Purpose.");
        EXPECT_EQ(status, Logger::LoggerStatus::FAILED_TO_OPEN);
        delete logger;
    }
}
TEST(BasicTest, TestCustomOperation) {
    Logger* logger =
        new Logger("INFO", std::string(getenv("HOME")) + "/" + std::string("test"), "logger_test");
    EXPECT_TRUE(logger->is_logger_ok());
    delete logger;
}
TEST(BasicTest, TestVerbosity) {
    eros_diagnostic::Diagnostic diag("UnitTestDevice",
                                     "UnitTestNode-Logger",
                                     System::MainSystem::SIMROVER,
                                     System::SubSystem::ENTIRE_SYSTEM,
                                     System::Component::ENTIRE_SUBSYSTEM);
    Logger* logger = new Logger("INFO", diag.node_name);
    EXPECT_TRUE(logger->is_logger_ok());
    EXPECT_TRUE(logger->set_logverbosity(Level::Type::INFO));
    EXPECT_EQ(logger->get_logverbosity(), Level::Type::INFO);
    EXPECT_TRUE(logger->set_logverbosity(Level::Type::WARN));
    EXPECT_EQ(logger->get_logverbosity(), Level::Type::WARN);
    EXPECT_TRUE(logger->set_logverbosity(Level::Type::ERROR));
    EXPECT_EQ(logger->get_logverbosity(), Level::Type::ERROR);
    EXPECT_FALSE(logger->set_logverbosity(Level::Type::UNKNOWN));
    EXPECT_EQ(logger->get_logverbosity(), Level::Type::ERROR);

    delete logger;
}
TEST(BasicTest, LogDiagnostic) {
    eros_diagnostic::Diagnostic diag("UnitTestDevice",
                                     "UnitTestNode-Logger",
                                     System::MainSystem::SIMROVER,
                                     System::SubSystem::ENTIRE_SYSTEM,
                                     System::Component::ENTIRE_SUBSYSTEM);
    Logger* logger = new Logger("DEBUG", diag.node_name);
    logger->enable_ROS_logger();
    eros_diagnostic::Diagnostic testDiagnostic;
    for (uint8_t i = 0; i <= (uint8_t)(Level::Type::END_OF_LIST); ++i) {
        testDiagnostic.level = (Level::Type)(i);
        EXPECT_EQ(logger->log_diagnostic(testDiagnostic), Logger::LoggerStatus::LOG_WRITTEN);
    }

    delete logger;
}
TEST(AdvancedTest, LogLineCount) {
    eros_diagnostic::Diagnostic diag("UnitTestDevice",
                                     "UnitTestNode-Logger",
                                     System::MainSystem::SIMROVER,
                                     System::SubSystem::ENTIRE_SYSTEM,
                                     System::Component::ENTIRE_SUBSYSTEM);
    Logger* logger = new Logger("DEBUG", diag.node_name);
    uint16_t linesToWrite = Logger::MAXLINE_COUNT + 100;
    logger->disable_consoleprint();  // Disabling to not annoy.
    for (uint16_t i = 0; i < linesToWrite; ++i) {
        logger->log_info(std::to_string(i + 1) + "/" + std::to_string(linesToWrite) +
                         ": A Long list of text to Log.");
    }

    delete logger;
}
int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}