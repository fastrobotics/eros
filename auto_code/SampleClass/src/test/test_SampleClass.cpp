/*! \file test_SampleClass.cpp
 */
#include <gtest/gtest.h>
#include <stdio.h>

#include "SampleClass.h"
using namespace eros;
using namespace SamplePackage;
TEST(BasicTest, TestDefintions) {
    // Test Type: Enum1
    for (uint8_t i = 0; i <= (uint8_t)(SampleClass::Enum1::END_OF_LIST); ++i) {
        if ((i == 0) || (i == (uint8_t)(SampleClass::Enum1::END_OF_LIST))) {
            EXPECT_TRUE(SampleClass::Enum1String((SampleClass::Enum1)(i)) == "UNKNOWN");
        }
        else {
            EXPECT_FALSE(SampleClass::Enum1String((SampleClass::Enum1)(i)) == "UNKNOWN");
        }
    }
}
TEST(BasicTest, TestOperation) {
    Logger* logger = new Logger("DEBUG", "SampleClass");
    SampleClass SUT;
    EXPECT_TRUE(SUT.init(logger));

    EXPECT_TRUE(SUT.update(10.0));

    EXPECT_TRUE(SUT.reset());

    EXPECT_TRUE(SUT.finish());
    delete logger;
}
int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}