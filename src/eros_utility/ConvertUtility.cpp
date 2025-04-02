#include <eros_utility/ConvertUtility.h>
namespace eros::eros_utility {

ros::Time ConvertUtility::convert_time(struct timeval t_) {
    ros::Time t;
    t.sec = t_.tv_sec;
    t.nsec = t_.tv_usec * 1000;
    return t;
}
ros::Time ConvertUtility::convert_time(double t_) {
    ros::Time t;
    t.sec = (int64_t)t_;
    double rem = t_ - (double)t.sec;
    t.nsec = (int64_t)(rem * 1000000.0);
    return t;
}

eros::armed_state ConvertUtility::convert(ArmDisarm::State v) {
    eros::armed_state msg;
    msg.armed_state = (uint8_t)v.state;
    return msg;
}
ArmDisarm::State ConvertUtility::convert(eros::armed_state v) {
    ArmDisarm::State data;
    data.state = (ArmDisarm::Type)v.armed_state;
    return data;
}
// No Obvious way to Unit test
// LCOV_EXCL_START
eros::ready_to_arm ConvertUtility::convert_fromptr(const eros::ready_to_arm::ConstPtr &t_ptr) {
    eros::ready_to_arm msg;
    msg.ready_to_arm = t_ptr->ready_to_arm;
    msg.diag = t_ptr->diag;
    return msg;
}
// LCOV_EXCL_STOP
// No Obvious way to Unit test
// LCOV_EXCL_START
eros::heartbeat ConvertUtility::convert_fromptr(const eros::heartbeat::ConstPtr &t_ptr) {
    eros::heartbeat msg;
    msg.stamp = t_ptr->stamp;
    msg.HostName = t_ptr->HostName;
    msg.BaseNodeName = t_ptr->BaseNodeName;
    msg.NodeName = t_ptr->NodeName;
    msg.NodeState = t_ptr->NodeState;
    return msg;
}
// LCOV_EXCL_STOP
// No Obvious way to Unit test
// LCOV_EXCL_START
eros::command ConvertUtility::convert_fromptr(const eros::command::ConstPtr &t_ptr) {
    eros::command cmd;
    cmd.Command = t_ptr->Command;
    cmd.CommandText = t_ptr->CommandText;
    cmd.Description = t_ptr->Description;
    cmd.Option1 = t_ptr->Option1;
    cmd.Option2 = t_ptr->Option2;
    cmd.Option3 = t_ptr->Option3;
    return cmd;
}
// LCOV_EXCL_STOP
// No Obvious way to Unit test
// LCOV_EXCL_START
eros::diagnostic ConvertUtility::convert_fromptr(const eros::diagnostic::ConstPtr &t_ptr) {
    eros::diagnostic diag;
    diag.Component = t_ptr->Component;
    diag.Description = t_ptr->Description;
    diag.DeviceName = t_ptr->DeviceName;
    diag.DiagnosticMessage = t_ptr->DiagnosticMessage;
    diag.DiagnosticType = t_ptr->DiagnosticType;
    diag.Level = t_ptr->Level;
    diag.NodeName = t_ptr->NodeName;
    diag.SubSystem = t_ptr->SubSystem;
    diag.System = t_ptr->System;
    return diag;
}
// LCOV_EXCL_STOP
// No Obvious way to Unit test
// LCOV_EXCL_START
eros::resource ConvertUtility::convert_fromptr(const eros::resource::ConstPtr &t_ptr) {
    eros::resource msg;
    msg.stamp = t_ptr->stamp;
    msg.Name = t_ptr->Name;
    msg.PID = t_ptr->PID;
    msg.CPU_Perc = t_ptr->CPU_Perc;
    msg.RAM_Perc = t_ptr->RAM_Perc;
    msg.DISK_Perc = t_ptr->DISK_Perc;
    return msg;
}
// LCOV_EXCL_STOP
// No Obvious way to Unit test
// LCOV_EXCL_START
eros::loadfactor ConvertUtility::convert_fromptr(const eros::loadfactor::ConstPtr &t_ptr) {
    eros::loadfactor msg;
    msg.stamp = t_ptr->stamp;
    msg.DeviceName = t_ptr->DeviceName;
    msg.loadfactor = t_ptr->loadfactor;
    return msg;
}
// LCOV_EXCL_STOP
// No Obvious way to Unit test
// LCOV_EXCL_START
eros::command_state ConvertUtility::convert_fromptr(const eros::command_state::ConstPtr &t_ptr) {
    eros::command_state msg;
    msg.stamp = t_ptr->stamp;
    msg.Name = t_ptr->Name;
    msg.CurrentCommand = t_ptr->CurrentCommand;
    msg.State = t_ptr->State;
    msg.PercentComplete = t_ptr->PercentComplete;
    msg.diag = t_ptr->diag;
    return msg;
}
// LCOV_EXCL_STOP
eros::armed_state ConvertUtility::convert_fromptr(const eros::armed_state::ConstPtr &t_ptr) {
    eros::armed_state msg;
    msg.armed_state = t_ptr->armed_state;
    return msg;
}
eros::mode_state ConvertUtility::convert_fromptr(const eros::mode_state::ConstPtr &t_ptr) {
    eros::mode_state msg;
    msg.mode_state = t_ptr->mode_state;
    return msg;
}
geometry_msgs::Twist ConvertUtility::convert_fromptr(const geometry_msgs::Twist::ConstPtr &t_ptr) {
    geometry_msgs::Twist msg;
    msg.linear.x = t_ptr->linear.x;
    msg.linear.y = t_ptr->linear.y;
    msg.linear.z = t_ptr->linear.z;
    msg.angular.x = t_ptr->angular.x;
    msg.angular.y = t_ptr->angular.y;
    msg.angular.z = t_ptr->angular.z;
    return msg;
}
eros::resource ConvertUtility::convert(eros::ResourceInfo res_info) {
    eros::resource res;
    res.stamp = res_info.stamp;
    res.Name = res_info.process_name;
    res.PID = res_info.pid;
    res.CPU_Perc = res_info.cpu_perc;
    res.RAM_Perc = res_info.ram_perc;
    res.DISK_Perc = res_info.disk_perc;
    return res;
}

sensor_msgs::Joy ConvertUtility::convert_fromptr(const sensor_msgs::Joy::ConstPtr &t_ptr) {
    sensor_msgs::Joy msg;
    msg.header = t_ptr->header;
    msg.axes = t_ptr->axes;
    msg.buttons = t_ptr->buttons;
    return msg;
}
}  // namespace eros::eros_utility