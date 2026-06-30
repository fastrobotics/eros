/*! \file test_core_utility.cpp
 */
#include <eros_utility/CoreUtility.h>
#include <gtest/gtest.h>

using namespace eros::eros_utility;

TEST(ExecTests, ExecCases) {
    {
        eros::ExecResult res = CoreUtility::exec("date", false);
        EXPECT_FALSE(res.AnyError);
        EXPECT_EQ(res.ErrorString, "");
        EXPECT_EQ(res.Result, "");

        res = CoreUtility::exec("date", true);
        EXPECT_FALSE(res.AnyError);
        EXPECT_EQ(res.ErrorString, "");
        EXPECT_NE(res.Result, "");
    }
}
TEST(UtilityEquality, TestCases) {
    {
        double a = 0.0;
        double b = 0.0;
        double eps = 0.1;
        EXPECT_TRUE(CoreUtility::isEqual(a, b, eps));
    }
    {
        double a = 0.0;
        double b = 10.0;
        double eps = 0.1;
        EXPECT_FALSE(CoreUtility::isEqual(a, b, eps));
    }
}
TEST(PrettyFunctions, PrettyCases) {
    {
        eros::file msg;
        EXPECT_GT(CoreUtility::pretty(msg).size(), 0);
    }
}
TEST(TestConversion, TimeChecks) {
    ros::Time::init();
    ros::Time t_now = ros::Time::now();
    double t_dur = CoreUtility::measure_time_diff(t_now, t_now);
    EXPECT_NEAR(t_dur, 0.0, 1e-8);
    sleep(1.0);
    ros::Time t_future = ros::Time::now();
    t_dur = CoreUtility::measure_time_diff(t_now, t_future);
    EXPECT_NEAR(t_dur, -1.0, 0.25);  // Allow for imprecise time clocks
    t_dur = CoreUtility::measure_time_diff(t_future, t_now);
    EXPECT_NEAR(t_dur, 1.0, 0.25);  // Allow for imprecise time clocks
}
int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}