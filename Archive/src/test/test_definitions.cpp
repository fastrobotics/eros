/*! \file test_definitions.cpp
 */
#include <eros/eros_Definitions.h>
#include <gtest/gtest.h>
#include <stdio.h>
using namespace eros;
TEST(BasicTest, TestDefinitions) {
    // Test Class: System
    {
        // Test Type: MainSystem
        for (uint8_t i = 0; i <= (uint8_t)(System::MainSystem::END_OF_LIST); ++i) {
            if ((i == 0) || (i == (uint8_t)(System::MainSystem::END_OF_LIST))) {
                EXPECT_TRUE(System::MainSystemString((System::MainSystem)(i)) == "UNKNOWN");
            }
            else {
                EXPECT_FALSE(System::MainSystemString((System::MainSystem)(i)) == "UNKNOWN");
            }
        }
        // Test Type: SubSystem
        for (uint8_t i = 0; i <= (uint8_t)(System::SubSystem::END_OF_LIST); ++i) {
            if ((i == 0) || (i == (uint8_t)(System::SubSystem::END_OF_LIST))) {
                EXPECT_TRUE(System::SubSystemString((System::SubSystem)(i)) == "UNKNOWN");
            }
            else {
                EXPECT_FALSE(System::SubSystemString((System::SubSystem)(i)) == "UNKNOWN");
            }
        }
        // Test Type: Component
        for (uint8_t i = 0; i <= (uint8_t)(System::Component::END_OF_LIST); ++i) {
            if ((i == 0) || (i == (uint8_t)(System::Component::END_OF_LIST))) {
                EXPECT_TRUE(System::ComponentString((System::Component)(i)) == "UNKNOWN");
            }
            else {
                EXPECT_FALSE(System::ComponentString((System::Component)(i)) == "UNKNOWN");
            }
        }
    }

    // Test Class: Level
    {
        // Test Type: Type
        for (uint8_t i = 0; i <= (uint8_t)(Level::Type::END_OF_LIST); ++i) {
            if ((i == 0) || (i == (uint8_t)(Level::Type::END_OF_LIST))) {
                EXPECT_TRUE(Level::LevelString((Level::Type)(i)) == "UNKNOWN");
                EXPECT_TRUE(Level::LevelType(Level::LevelString((Level::Type)(i))) ==
                            Level::Type::UNKNOWN);
            }
            else {
                EXPECT_FALSE(Level::LevelString((Level::Type)(i)) == "UNKNOWN");
                EXPECT_TRUE(Level::LevelType(Level::LevelString((Level::Type)(i))) ==
                            (Level::Type)(i));
            }
        }
    }

    // Test Class: ArmDisarm
    {
        // Test Type: Type
        for (uint8_t i = 0; i <= (uint8_t)(ArmDisarm::Type::END_OF_LIST); ++i) {
            if ((i == 0) || (i == (uint8_t)(ArmDisarm::Type::END_OF_LIST))) {
                EXPECT_TRUE(ArmDisarm::ArmDisarmString((ArmDisarm::Type)(i)) == "UNKNOWN");
            }
            else {
                EXPECT_FALSE(ArmDisarm::ArmDisarmString((ArmDisarm::Type)(i)) == "UNKNOWN");
            }
        }
    }

    // Test Class: Command
    {
        // Test Type: Type
        for (uint8_t i = 0; i <= (uint8_t)(Command::Type::END_OF_LIST); ++i) {
            if ((i == 0) || (i == (uint8_t)(Command::Type::END_OF_LIST))) {
                EXPECT_TRUE(Command::CommandString((Command::Type)(i)) == "UNKNOWN");
            }
            else {
                EXPECT_FALSE(Command::CommandString((Command::Type)(i)) == "UNKNOWN");
            }
        }
    }

    // Test Class: Node
    {
        // Test Type: State
        for (uint8_t i = 0; i <= (uint8_t)(Node::State::END_OF_LIST); ++i) {
            if ((i == 0) || (i == (uint8_t)(Node::State::END_OF_LIST))) {
                EXPECT_TRUE(Node::NodeStateString((Node::State)(i)) == "UNKNOWN");
                EXPECT_TRUE(Node::NodeState(Node::NodeStateString((Node::State)(i))) ==
                            Node::State::UNKNOWN);
            }
            else {
                EXPECT_FALSE(Node::NodeStateString((Node::State)(i)) == "UNKNOWN");
                EXPECT_TRUE(Node::NodeState(Node::NodeStateString((Node::State)(i))) ==
                            (Node::State)(i));
            }
        }
    }

    // Test Class: Architecture
    // Test Type: Type
    for (uint8_t i = 0; i <= (uint8_t)(Architecture::Type::END_OF_LIST); ++i) {
        if ((i == 0) || (i == (uint8_t)(Architecture::Type::END_OF_LIST))) {
            EXPECT_TRUE(Architecture::ArchitectureString((Architecture::Type)(i)) == "UNKNOWN");
            EXPECT_TRUE(Architecture::ArchitectureType(Architecture::ArchitectureString(
                            (Architecture::Type)(i))) == Architecture::Type::UNKNOWN);
        }
        else {
            EXPECT_FALSE(Architecture::ArchitectureString((Architecture::Type)(i)) == "UNKNOWN");
            EXPECT_TRUE(Architecture::ArchitectureType(Architecture::ArchitectureString(
                            (Architecture::Type)(i))) == (Architecture::Type)(i));
        }
    }
}
int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}