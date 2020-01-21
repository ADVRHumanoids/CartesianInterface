#include "fmt/format.h"
#include "CartesianRos.h"
#include <cartesian_interface/GetCartesianTaskInfo.h>
#include <cartesian_interface/SetBaseLink.h>
#include <cartesian_interface/SetControlMode.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <eigen_conversions/eigen_msg.h>

using namespace XBot::Cartesian;
using namespace XBot::Cartesian::ClientApi;
using namespace cartesian_interface;

CartesianRos::CartesianRos(std::string name,
                           ros::NodeHandle nh):
    TaskRos(name, nh)
{
    _cart_info_cli = _nh.serviceClient<GetCartesianTaskInfo>(name + "/get_cartesian_task_properties");
    if(!_cart_info_cli.waitForExistence(ros::Duration(1.0)) || !_cart_info_cli.exists())
    {
        throw std::runtime_error(fmt::format("Non existent service '{}'",
                                             _cart_info_cli.getService()));
    }

    _set_base_link_cli = _nh.serviceClient<SetBaseLink>(name + "/set_base_link");

    _set_ctrl_mode_cli = _nh.serviceClient<SetControlMode>(name + "/set_control_mode");

    _pose_ref_pub = _nh.advertise<geometry_msgs::PoseStamped>(name + "/reference", 1);

    _vel_ref_pub = _nh.advertise<geometry_msgs::TwistStamped>(name + "/velocity_reference", 1);

    auto on_ref_recv = [this](geometry_msgs::PoseStampedConstPtr msg)
    {
        tf::poseMsgToEigen(msg->pose, _Tref);
    };

    _pose_ref_sub = _nh.subscribe<geometry_msgs::PoseStamped>(name + "/current_reference", 1,
                                                              on_ref_recv);

    auto on_vref_recv = [this](geometry_msgs::TwistStampedConstPtr msg)
    {
        tf::twistMsgToEigen(msg->twist, _vref);
    };

    _pose_ref_sub = _nh.subscribe<geometry_msgs::TwistStamped>(name + "/current_velocity_reference", 1,
                                                              on_vref_recv);
}

void CartesianRos::enableOnlineTrajectoryGeneration()
{
}

bool CartesianRos::isSubtaskLocal() const
{
    return get_task_info().use_body_jacobian;
}

void CartesianRos::getVelocityLimits(double & max_vel_lin, double & max_vel_ang) const
{
}

void CartesianRos::getAccelerationLimits(double & max_acc_lin, double & max_acc_ang) const
{
}

void CartesianRos::setVelocityLimits(double max_vel_lin, double max_vel_ang)
{
}

void CartesianRos::setAccelerationLimits(double max_acc_lin, double max_acc_ang)
{
}

State CartesianRos::getTaskState() const
{
    return StringToEnum<State>(get_task_info().state);
}

const std::string & CartesianRos::getBaseLink() const
{
    _base_link = get_task_info().base_link;
    return _base_link;
}

bool CartesianRos::setBaseLink(const std::string & new_base_link)
{
    SetBaseLink srv;
    srv.request.base_link = new_base_link;

    if(!_set_base_link_cli.call(srv))
    {
        throw std::runtime_error(fmt::format("Unable to call service '{}'",
                                             _set_base_link_cli.getService()));
    }

    ROS_INFO("%s", srv.response.message.c_str());

    return srv.response.success;
}

const std::string & CartesianRos::getDistalLink() const
{
    _distal_link = get_task_info().distal_link;
    return _distal_link;
}

ControlType CartesianRos::getControlMode() const
{
    return StringToEnum<ControlType>(get_task_info().control_mode);
}

bool CartesianRos::setControlMode(const ControlType & value)
{
    SetControlMode srv;
    srv.request.ctrl_mode = EnumToString(value);

    if(!_set_ctrl_mode_cli.call(srv))
    {
        throw std::runtime_error(fmt::format("Unable to call service '{}'",
                                             _set_ctrl_mode_cli.getService()));
    }

    ROS_INFO("%s", srv.response.message.c_str());

    return srv.response.success;
}

bool CartesianRos::getCurrentPose(Eigen::Affine3d & base_T_ee) const
{
}

bool CartesianRos::getPoseReference(Eigen::Affine3d & base_T_ref, Eigen::Vector6d * base_vel_ref, Eigen::Vector6d * base_acc_ref) const
{
    base_T_ref = _Tref;
    if(base_vel_ref) *base_vel_ref = _vref;
    return true;
}

bool CartesianRos::getPoseReferenceRaw(Eigen::Affine3d & base_T_ref, Eigen::Vector6d * base_vel_ref, Eigen::Vector6d * base_acc_ref) const
{
}

bool CartesianRos::setPoseReference(const Eigen::Affine3d & base_T_ref)
{
    geometry_msgs::PoseStamped msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "";
    tf::poseEigenToMsg(base_T_ref, msg.pose);
    _pose_ref_pub.publish(msg);

    return true;
}

bool CartesianRos::setPoseReferenceRaw(const Eigen::Affine3d & base_T_ref)
{
}

bool CartesianRos::setVelocityReference(const Eigen::Vector6d & base_vel_ref)
{
}

bool CartesianRos::getPoseTarget(Eigen::Affine3d & base_T_ref) const
{
}

bool CartesianRos::setPoseTarget(const Eigen::Affine3d & base_T_ref, double time)
{
}

int CartesianRos::getCurrentSegmentId() const
{
}

bool CartesianRos::setWayPoints(const Trajectory::WayPointVector & way_points)
{
}

void CartesianRos::abort()
{
}

GetCartesianTaskInfoResponse CartesianRos::get_task_info() const
{
    cartesian_interface::GetCartesianTaskInfo srv;
    if(!_cart_info_cli.call(srv))
    {
        throw std::runtime_error(fmt::format("Unable to call service '{}'",
                                             _cart_info_cli.getService()));
    }

    return srv.response;
}
