#include "fmt/format.h"
#include "CartesianRos.h"
#include <cartesian_interface/GetCartesianTaskInfo.h>
#include <cartesian_interface/SetBaseLink.h>
#include <cartesian_interface/SetControlMode.h>
#include <cartesian_interface/SetSafetyLimits.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <eigen_conversions/eigen_msg.h>

using namespace XBot::Cartesian;
using namespace XBot::Cartesian::ClientApi;
using namespace cartesian_interface;

CartesianRos::CartesianRos(std::string name,
                           ros::NodeHandle nh):
    TaskRos(name, nh),
    _action_cli(nh, name + "/reach", false),
    _Tref_recv(false), _vref_recv(false)
{
    _cart_info_cli = _nh.serviceClient<GetCartesianTaskInfo>(name + "/get_cartesian_task_properties");
    if(!_cart_info_cli.waitForExistence(ros::Duration(1.0)) || !_cart_info_cli.exists())
    {
        throw std::runtime_error(fmt::format("Non existent service '{}'",
                                             _cart_info_cli.getService()));
    }

    if(!_action_cli.isServerConnected())
    {
//        throw std::runtime_error(fmt::format("Unable to reach action server '{}'",
//                                             nh.resolveName(name + "/reach")));
    }

    _set_base_link_cli = _nh.serviceClient<SetBaseLink>(name + "/set_base_link");

    _set_ctrl_mode_cli = _nh.serviceClient<SetControlMode>(name + "/set_control_mode");

    _pose_ref_pub = _nh.advertise<geometry_msgs::PoseStamped>(name + "/reference", 1);

    _vel_ref_pub = _nh.advertise<geometry_msgs::TwistStamped>(name + "/velocity_reference", 1);

    auto on_ref_recv = [this](geometry_msgs::PoseStampedConstPtr msg)
    {
        _Tref_recv = true;
        tf::poseMsgToEigen(msg->pose, _Tref);
    };

    _pose_ref_sub = _nh.subscribe<geometry_msgs::PoseStamped>(name + "/current_reference", 1,
                                                              on_ref_recv);

    auto on_vref_recv = [this](geometry_msgs::TwistStampedConstPtr msg)
    {
        _vref_recv = true;
        tf::twistMsgToEigen(msg->twist, _vref);
    };

    _vel_ref_sub = _nh.subscribe<geometry_msgs::TwistStamped>(name + "/current_velocity_reference", 1,
                                                              on_vref_recv);

    _set_safety_lims_cli = _nh.serviceClient<SetSafetyLimits>(name + "/set_safety_limits");
}

bool CartesianRos::validate()
{
    return TaskRos::validate() && _Tref_recv && _vref_recv;
}

void CartesianRos::enableOnlineTrajectoryGeneration()
{
}

bool CartesianRos::isSubtaskLocal() const
{
    return get_task_info().use_body_jacobian;
}

void CartesianRos::getVelocityLimits(double & max_vel_lin,
                                     double & max_vel_ang) const
{
    auto info = get_task_info();
    max_vel_lin = info.max_vel_lin;
    max_vel_ang = info.max_vel_ang;
}

void CartesianRos::getAccelerationLimits(double & max_acc_lin,
                                         double & max_acc_ang) const
{
    auto info = get_task_info();
    max_acc_lin = info.max_acc_lin;
    max_acc_ang = info.max_acc_ang;
}

void CartesianRos::setVelocityLimits(double max_vel_lin,
                                     double max_vel_ang)
{
    SetSafetyLimits srv;
    srv.request.max_vel_lin = max_vel_lin;
    srv.request.max_vel_ang = max_vel_ang;
    srv.request.max_acc_lin = -1.0;
    srv.request.max_acc_ang = -1.0;

    if(!_set_safety_lims_cli.call(srv))
    {
        throw std::runtime_error(fmt::format("Unable to call service '{}'",
                                             _set_safety_lims_cli.getService()));
    }

    ROS_INFO("%s", srv.response.message.c_str());
}

void CartesianRos::setAccelerationLimits(double max_acc_lin,
                                         double max_acc_ang)
{
    SetSafetyLimits srv;
    srv.request.max_vel_lin = -1.0;
    srv.request.max_vel_ang = -1.0;
    srv.request.max_acc_lin = max_acc_lin;
    srv.request.max_acc_ang = max_acc_ang;

    if(!_set_safety_lims_cli.call(srv))
    {
        throw std::runtime_error(fmt::format("Unable to call service '{}'",
                                             _set_safety_lims_cli.getService()));
    }

    ROS_INFO("%s", srv.response.message.c_str());
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

bool CartesianRos::getCurrentPose(Eigen::Affine3d& base_T_ee) const
{
    throw std::runtime_error(fmt::format("Unsupported function '{}'",
                                         __PRETTY_FUNCTION__));
}

bool CartesianRos::getPoseReference(Eigen::Affine3d& base_T_ref,
                                    Eigen::Vector6d* base_vel_ref,
                                    Eigen::Vector6d* base_acc_ref) const
{
    base_T_ref = _Tref;
    if(base_vel_ref) *base_vel_ref = _vref;
    if(base_acc_ref) base_acc_ref->setZero();
    return true;
}

bool CartesianRos::getPoseReferenceRaw(Eigen::Affine3d& base_T_ref, Eigen::Vector6d * base_vel_ref, Eigen::Vector6d * base_acc_ref) const
{
    throw std::runtime_error(fmt::format("Unsupported function '{}'",
                                         __PRETTY_FUNCTION__));
}

bool CartesianRos::setPoseReference(const Eigen::Affine3d& base_T_ref)
{
    geometry_msgs::PoseStamped msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "";
    tf::poseEigenToMsg(base_T_ref, msg.pose);
    _pose_ref_pub.publish(msg);

    return true;
}

bool CartesianRos::setPoseReferenceRaw(const Eigen::Affine3d& base_T_ref)
{
    throw std::runtime_error(fmt::format("Unsupported function '{}'",
                                         __PRETTY_FUNCTION__));
}

bool CartesianRos::setVelocityReference(const Eigen::Vector6d& base_vel_ref)
{
    return setVelocityReference(base_vel_ref, "");
}

bool CartesianRos::setVelocityReference(const Eigen::Vector6d& base_vel_ref,
                                        const std::string& base_frame)
{
    geometry_msgs::TwistStamped msg;
    msg.header.frame_id = base_frame;
    msg.header.stamp = ros::Time::now();
    tf::twistEigenToMsg(base_vel_ref, msg.twist);
    _vel_ref_pub.publish(msg);

    return true;
}

bool CartesianRos::getPoseTarget(Eigen::Affine3d& base_T_ref) const
{
    throw std::runtime_error(fmt::format("Unsupported function '{}'",
                                         __PRETTY_FUNCTION__));
}

bool CartesianRos::setPoseTarget(const Eigen::Affine3d& base_T_ref,
                                 double time)
{
    Trajectory::WayPoint wp;
    wp.time = time;
    wp.frame = base_T_ref;

    return setWayPoints({wp});
}

int CartesianRos::getCurrentSegmentId() const
{
    return _current_segment_idx;
}

bool CartesianRos::setWayPoints(const Trajectory::WayPointVector& way_points)
{
    return setWayPoints(way_points, false);
}

void CartesianRos::abort()
{
    _action_cli.cancelAllGoals();
}

bool CartesianRos::waitReachCompleted(double timeout)
{
    return _action_cli.waitForResult(ros::Duration(timeout));
}

bool CartesianRos::setWayPoints(const Trajectory::WayPointVector& way_points,
                                bool incremental)
{
    cartesian_interface::ReachPoseGoal goal;
    goal.incremental = incremental;

    for(const auto& f : way_points)
    {
        geometry_msgs::Pose frame;
        tf::poseEigenToMsg(f.frame, frame);

        goal.frames.push_back(frame);
        goal.time.push_back(f.time);
    }

//    if(!_action_cli.waitForServer(ros::Duration(2.0)))
//    {
//        throw std::runtime_error(fmt::format("Unable to reach action server '{}'",
//                                             _nh.resolveName(getName() + "/reach")));
//    }

    _action_cli.sendGoal(goal,
                         boost::bind(&CartesianRos::on_action_done, this, _1, _2),
                         boost::bind(&CartesianRos::on_action_active, this),
                         boost::bind(&CartesianRos::on_reach_feedback_recv, this, _1));
    return true;
}

GetCartesianTaskInfoResponse CartesianRos::get_task_info() const
{
    if(asyncMode())
    {
        GetCartesianTaskInfoResponse res;
        res.base_link         = _info.base_link        ;
        res.distal_link       = _info.distal_link      ;
        res.control_mode      = _info.control_mode     ;
        res.state             = _info.state            ;
        res.use_body_jacobian = _info.use_body_jacobian;
        res.max_vel_lin       = _info.max_vel_lin      ;
        res.max_vel_ang       = _info.max_vel_ang      ;
        res.max_acc_lin       = _info.max_acc_lin      ;
        res.max_acc_ang       = _info.max_acc_ang      ;

        return res;
    }

    cartesian_interface::GetCartesianTaskInfo srv;
    if(!_cart_info_cli.call(srv))
    {
        throw std::runtime_error(fmt::format("Unable to call service '{}'",
                                             _cart_info_cli.getService()));
    }

    return srv.response;
}

void CartesianRos::on_reach_feedback_recv(const ReachPoseFeedbackConstPtr & feedback)
{
    _current_segment_idx = feedback->current_segment_id;
}

void CartesianRos::on_action_active()
{
    ROS_INFO("Reach action for task '%s' has become active",
             getName().c_str());
}

void CartesianRos::on_action_done(const actionlib::SimpleClientGoalState & state,
                                  const ReachPoseResultConstPtr & result)
{
    ROS_INFO("Reach action for task '%s' has been completed",
             getName().c_str());

    _current_segment_idx = -1;
}


void CartesianRos::registerObserver(TaskObserver::WeakPtr obs)
{
    if(auto cobs = std::dynamic_pointer_cast<CartesianTaskObserver>(obs.lock()))
    {
        _observers.push_back(cobs);
    }
}

void CartesianRos::notifyTaskChanged(const std::string & message)
{
    TaskRos::notifyTaskChanged(message);

    if(message == "BaseLink")
    {
        NOTIFY_OBSERVERS(BaseLink);
    }
    else if(message == "ControlMode")
    {
        NOTIFY_OBSERVERS(ControlMode);
    }
}
