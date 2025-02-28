/*! \file ConvertUtility.h
 */
#pragma once
// ROS Messages
#include <eros/armed_state.h>
#include <eros/command.h>
#include <eros/command_state.h>
#include <eros/diagnostic.h>
#include <eros/eros_Definitions.h>
#include <eros/file.h>
#include <eros/heartbeat.h>
#include <eros/loadfactor.h>
#include <eros/mode_state.h>
#include <eros/ready_to_arm.h>
#include <eros/resource.h>
#include <eros/uptime.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>
#include <time.h>

#include <string>

namespace eros::eros_utility {
class ConvertUtility
{
   public:
    //! Convert struct timeval to ros::Time
    /*!
      \param t Standard timeval object
      \return Time converted to ros::Time
    */
    static ros::Time convert_time(struct timeval t);

    //! Convert time as a float to ros::Time
    /*!
      \param t timestamp in seconds.
      \return Time converted to ros::Time
    */
    static ros::Time convert_time(double t);

    //! Convert eros::command message (as received via a ROS Node) to the regular datatype
    /*!
      \param t_ptr The pointer to the object
      \return The object
    */
    static eros::command convert_fromptr(const eros::command::ConstPtr& t_ptr);
    static eros::ready_to_arm convert_fromptr(const eros::ready_to_arm::ConstPtr& t_ptr);
    static eros::command_state convert_fromptr(const eros::command_state::ConstPtr& t_ptr);

    //! Convert eros::diagnostic message (as received via a ROS Node) to the regular datatype
    /*!
      \param t_ptr The pointer to the object
      \return The object
    */
    static eros::diagnostic convert_fromptr(const eros::diagnostic::ConstPtr& t_ptr);

    static eros::armed_state convert(ArmDisarm::State v);
    static ArmDisarm::State convert(eros::armed_state v);
    //! Convert eros::heartbeat message (as received via a ROS Node) to the regular datatype
    /*!
      \param t_ptr The pointer to the object
      \return The object
    */
    static eros::heartbeat convert_fromptr(const eros::heartbeat::ConstPtr& t_ptr);
    //! Convert eros::resource message (as received via a ROS Node) to the regular datatype
    /*!
      \param t_ptr The pointer to the object
      \return The object
    */
    static eros::resource convert_fromptr(const eros::resource::ConstPtr& t_ptr);
    //! Convert eros::loadfactor message (as received via a ROS Node) to the regular datatype
    /*!
      \param t_ptr The pointer to the object
      \return The object
    */
    static eros::loadfactor convert_fromptr(const eros::loadfactor::ConstPtr& t_ptr);
    static eros::resource convert(eros::ResourceInfo res_info);
    static eros::armed_state convert_fromptr(const eros::armed_state::ConstPtr& t_ptr);
    static eros::mode_state convert_fromptr(const eros::mode_state::ConstPtr& t_ptr);
    static geometry_msgs::Twist convert_fromptr(const geometry_msgs::Twist::ConstPtr& t_ptr);
};
}  // namespace eros::eros_utility
