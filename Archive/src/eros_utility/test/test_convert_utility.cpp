/*! \file test_convert_utility.cpp
 */
#include <eros_utility/ConvertUtility.h>
#include <gtest/gtest.h>

using namespace eros::eros_utility;
TEST(TestConversion, ConvertTime) {
    {
        double t_in = 0.0;
        ros::Time t = ConvertUtility::convert_time(t_in);
        EXPECT_EQ(t.sec, 0);
        EXPECT_EQ(t.nsec, 0);
    }
    {
        struct timeval t_in;
        t_in.tv_sec = 0;
        t_in.tv_usec = 0;
        ros::Time t = ConvertUtility::convert_time(t_in);
        EXPECT_EQ(t.sec, 0);
        EXPECT_EQ(t.nsec, 0);
    }
}
TEST(TestConversion, ArmedState) {
    {
        eros::ArmDisarm::State in;
        in.state = eros::ArmDisarm::Type::ARMED;
        eros::armed_state out = ConvertUtility::convert(in);
        EXPECT_EQ((uint8_t)in.state, (uint8_t)out.armed_state);
    }
    {
        eros::armed_state in;
        in.armed_state = (uint8_t)eros::ArmDisarm::Type::ARMED;
        eros::ArmDisarm::State out = ConvertUtility::convert(in);
        EXPECT_EQ((uint8_t)in.armed_state, (uint8_t)out.state);
    }
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}