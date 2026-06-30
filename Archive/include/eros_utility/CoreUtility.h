/*! \file CoreUtility.h
 */
#pragma once

#include <eros/eros_Definitions.h>
#include <eros/file.h>
#include <math.h>
#include <stdio.h>

#include <string>

//! Enhanced-ROS Utility Namespace
namespace eros::eros_utility {
class CoreUtility
{
   public:
    static bool isEqual(double a, double b, double eps);

    //! Execute a command
    /*!
      \param cmd The command to execute
      \param wait_for_results If function should return results or not.
      \return The result of the command
    */
    static eros::ExecResult exec(const char* cmd, bool wait_for_result);

    static std::string pretty(eros::file msg);
    /*! \brief Measures time delay between 2 ros::Time timestamps.
     *  Generally, if wanting to measure the time from now to a previous mark,
     * the current timestamp should be the first parameter and the previous mark should be the 2nd
     * parameter.
     */
    static double measure_time_diff(ros::Time time_a, ros::Time time_b);
};
}  // namespace eros::eros_utility