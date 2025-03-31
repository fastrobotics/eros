/*! \file test_{{cookiecutter.class_name}}.cpp
 */
#include <gtest/gtest.h>
#include <stdio.h>
#include "{{cookiecutter.class_name}}.h"
using namespace eros;
using namespace {{cookiecutter.package_name}}
TEST(BasicTest, TestDefintions) {
    // Test Type: Enum1
    for (uint8_t i = 0; i <= (uint8_t)({{cookiecutter.class_name}}::Enum1::END_OF_LIST); ++i) {

        if ((i == 0) || (i == (uint8_t)({{cookiecutter.class_name}}::Enum1::END_OF_LIST))) {
            EXPECT_TRUE({{cookiecutter.class_name}}::Enum1String(({{cookiecutter.class_name}}::Enum1)(i)) == "UNKNOWN");
        }
        else {
            EXPECT_FALSE({{cookiecutter.class_name}}::Enum1String(({{cookiecutter.class_name}}::Enum1)(i)) == "UNKNOWN");
        }
    }
}
TEST(BasicTest, TestOperation) {
    Logger* logger = new Logger("DEBUG", "{{cookiecutter.class_name}}");
    {{cookiecutter.class_name}} SUT;
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