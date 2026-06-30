/*! \file test_diagnostic.cpp
 */
#include <eros_diagnostic/Diagnostic.h>
#include <eros_diagnostic/DiagnosticUtility.h>
#include <gtest/gtest.h>
#include <stdio.h>
using namespace eros::eros_diagnostic;
TEST(BasicTest, TestDefintions) {
    // Test Type: DiagnosticType
    for (uint8_t i = 1; i < (uint8_t)(DiagnosticType::END_OF_LIST); ++i) {
        EXPECT_FALSE(DiagnosticUtility::DiagnosticTypeString((DiagnosticType)(i)) == "UNKNOWN");
    }
    // Test Type: Message
    for (uint8_t i = 1; i < (uint8_t)(Message::END_OF_LIST); ++i) {
        EXPECT_FALSE(DiagnosticUtility::DiagnosticMessageString((Message)(i)) == "UNKNOWN");
    }
}
int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}