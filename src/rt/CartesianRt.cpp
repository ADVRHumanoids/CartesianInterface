#include "rt/CartesianRt.h"

using namespace XBot::Cartesian;
namespace pl = std::placeholders;

CartesianRt::CartesianRt(CartesianTask::Ptr task):
    TaskRt(task),
    _task_impl(task)
{
    CartesianRt::sendState(false);
    _to_cli_queue.reset(_rt_data);
    CartesianRt::sendState(true);
    CartesianRt::update(0, 0);
}

void CartesianRt::callAvailable()
{
    TaskRt::callAvailable();

    _cb_queue.consume_all([this](CallbackType& cb)
    {
        cb(*_task_impl);
    });
}

void CartesianRt::sendState(bool send)
{
    TaskRt::sendState(send);

    _task_impl->getPoseReference(_rt_data._T,
                                 &_rt_data._vel, &_rt_data._acc);

    _rt_data._base_link = _task_impl->getBaseLink();

    _rt_data._state = _task_impl->getTaskState();

    _task_impl->getVelocityLimits(_rt_data._max_vel.first,
                                  _rt_data._max_vel.second);

    _task_impl->getAccelerationLimits(_rt_data._max_acc.first,
                                      _rt_data._max_acc.second);

    _task_impl->getCurrentPose(_rt_data._Tcurr);

    _rt_data._ctrl_mode = _task_impl->getControlMode();

    _rt_data._distal_link = _task_impl->getDistalLink();

    _rt_data._trj_segment = _task_impl->getCurrentSegmentId();

    _rt_data._is_body_jacobian = _task_impl->isSubtaskLocal();

    _to_cli_queue.push(_rt_data);
}

void CartesianRt::enableOnlineTrajectoryGeneration()
{
}

bool CartesianRt::isSubtaskLocal() const
{
    return _cli_data._is_body_jacobian;
}

void CartesianRt::getVelocityLimits(double & max_vel_lin, double & max_vel_ang) const
{
    max_vel_lin = _cli_data._max_vel.first;
    max_vel_ang = _cli_data._max_vel.second;
}

void CartesianRt::getAccelerationLimits(double & max_acc_lin, double & max_acc_ang) const
{
    max_acc_lin = _cli_data._max_acc.first;
    max_acc_ang = _cli_data._max_acc.second;
}

void CartesianRt::setVelocityLimits(double max_vel_lin, double max_vel_ang)
{
    auto cb = std::bind(&CartesianTask::setVelocityLimits,
                        pl::_1, max_vel_lin, max_vel_ang);

    _cb_queue.push(cb);
}

void CartesianRt::setAccelerationLimits(double max_acc_lin, double max_acc_ang)
{
    auto cb = std::bind(&CartesianTask::setAccelerationLimits,
                        pl::_1, max_acc_lin, max_acc_ang);

    _cb_queue.push(cb);
}

State CartesianRt::getTaskState() const
{
    return _rt_data._state;
}

const std::string & CartesianRt::getBaseLink() const
{
    return _rt_data._base_link;
}

bool CartesianRt::setBaseLink(const std::string & new_base_link)
{
    auto cb = std::bind(&CartesianTask::setBaseLink,
                        pl::_1, new_base_link);

    return _cb_queue.push(cb);
}

const std::string & CartesianRt::getDistalLink() const
{
    return _cli_data._distal_link;
}

ControlType CartesianRt::getControlMode() const
{
    return _cli_data._ctrl_mode;
}

bool CartesianRt::setControlMode(const ControlType & value)
{
    auto cb = std::bind(&CartesianTask::setControlMode,
                        pl::_1, value);

    return _cb_queue.push(cb);
}

bool CartesianRt::getCurrentPose(Eigen::Affine3d & base_T_ee) const
{
    base_T_ee = _cli_data._Tcurr;
    return true;
}

bool CartesianRt::getPoseReference(Eigen::Affine3d & base_T_ref,
                                   Eigen::Vector6d * base_vel_ref,
                                   Eigen::Vector6d * base_acc_ref) const
{
    base_T_ref = _cli_data._T;
    if(base_vel_ref) *base_vel_ref = _cli_data._vel;
    if(base_acc_ref) *base_acc_ref = _cli_data._acc;
    return true;
}

bool CartesianRt::getPoseReferenceRaw(Eigen::Affine3d & base_T_ref,
                                      Eigen::Vector6d * base_vel_ref,
                                      Eigen::Vector6d * base_acc_ref) const
{
    base_T_ref = _cli_data._T;
    if(base_vel_ref) *base_vel_ref = _cli_data._vel;
    if(base_acc_ref) *base_acc_ref = _cli_data._acc;
    return true;
}

bool CartesianRt::setPoseReference(const Eigen::Affine3d & base_T_ref)
{
    auto cb = std::bind(&CartesianTask::setPoseReference,
                        pl::_1, base_T_ref);

    return _cb_queue.push(cb);
}

bool CartesianRt::setPoseReferenceRaw(const Eigen::Affine3d & base_T_ref)
{
    auto cb = std::bind(&CartesianTask::setPoseReferenceRaw,
                        pl::_1, base_T_ref);

    return _cb_queue.push(cb);
}

bool CartesianRt::setGains(const Eigen::Matrix6d& Kp, const Eigen::Matrix6d& Kd)
{
    auto cb = std::bind(&CartesianTask::setGains,
                        pl::_1, Kp, Kd);

    return _cb_queue.push(cb);
}

bool CartesianRt::setVelocityReference(const Eigen::Vector6d & base_vel_ref)
{
    auto cb = std::bind(&CartesianTask::setVelocityReference,
                        pl::_1, base_vel_ref);

    return _cb_queue.push(cb);
}

bool CartesianRt::getPoseTarget(Eigen::Affine3d & base_T_ref) const
{
    return false;
}

bool CartesianRt::setPoseTarget(const Eigen::Affine3d & base_T_ref, double time)
{
    auto cb = std::bind(&CartesianTask::setPoseTarget,
                        pl::_1, base_T_ref, time);

    return _cb_queue.push(cb);
}

int CartesianRt::getCurrentSegmentId() const
{
    return _cli_data._trj_segment;
}

bool CartesianRt::setWayPoints(const Trajectory::WayPointVector & way_points)
{
    auto cb = std::bind(&CartesianTask::setWayPoints,
                        pl::_1, way_points);

    return _cb_queue.push(cb);
}

void CartesianRt::abort()
{
    auto cb = std::bind(&CartesianTask::abort,
                        pl::_1);

    _cb_queue.push(cb);
}


void XBot::Cartesian::CartesianRt::update(double time, double period)
{
    TaskRt::update(time, period);
     while(_to_cli_queue.pop(_cli_data));
}
