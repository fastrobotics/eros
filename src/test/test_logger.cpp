/*! \file test_logger.cpp
 */
#include <eros/Logger.h>
#include <gtest/gtest.h>
#include <stdio.h>
using namespace eros;
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
    Logger* logger = new Logger("INFO", "UnitTestNode-Logger");
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
TEST(AdvancedTest, LogLineCount) {
    Logger* logger = new Logger("DEBUG", "UnitTestNode-Logger");
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