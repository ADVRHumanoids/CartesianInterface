#ifndef CARTESIANRT_H
#define CARTESIANRT_H

#include <cartesian_interface/problem/Cartesian.h>
#include <cartesian_interface/sdk/rt/TaskRt.h>

namespace XBot { namespace Cartesian {


class CartesianRt : public virtual CartesianTask,
        public TaskRt
{

public:

    CartesianRt(CartesianTask::Ptr task);

    // TaskRt interface
public:

    void callAvailable() override;
    void sendState(bool send) override;

    // CartesianTask interface
public:

    void enableOnlineTrajectoryGeneration() override;
    bool isSubtaskLocal() const override;
    void getVelocityLimits(double & max_vel_lin, double & max_vel_ang) const override;
    void getAccelerationLimits(double & max_acc_lin, double & max_acc_ang) const override;
    void setVelocityLimits(double max_vel_lin, double max_vel_ang) override;
    void setAccelerationLimits(double max_acc_lin, double max_acc_ang) override;
    State getTaskState() const override;
    const std::string & getBaseLink() const override;
    bool setBaseLink(const std::string & new_base_link) override;
    const std::string & getDistalLink() const override;
    ControlType getControlMode() const override;
    bool setControlMode(const ControlType & value) override;
    bool getCurrentPose(Eigen::Affine3d & base_T_ee) const override;
    bool getPoseReference(Eigen::Affine3d & base_T_ref, Eigen::Vector6d * base_vel_ref, Eigen::Vector6d * base_acc_ref) const override;
    bool getPoseReferenceRaw(Eigen::Affine3d & base_T_ref, Eigen::Vector6d * base_vel_ref, Eigen::Vector6d * base_acc_ref) const override;
    bool setPoseReference(const Eigen::Affine3d & base_T_ref) override;
    bool setPoseReferenceRaw(const Eigen::Affine3d & base_T_ref) override;
    bool setVelocityReference(const Eigen::Vector6d & base_vel_ref) override;
    bool setAccelerationReference(const Eigen::Vector6d & base_acc_ref) override;
    bool getPoseTarget(Eigen::Affine3d & base_T_ref) const override;
    bool setPoseTarget(const Eigen::Affine3d & base_T_ref, double time) override;
    int getCurrentSegmentId() const override;
    bool setWayPoints(const Trajectory::WayPointVector & way_points) override;
    void abort() override;

private:

    typedef std::function<void(CartesianTask&)> CallbackType;
    LockFreeQueue<CallbackType, 1024> _cb_queue;

    struct DataToClient
    {
        ControlType _ctrl_mode;
        std::string _base_link, _distal_link;
        bool _is_body_jacobian;
        double _orientation_gain;
        Eigen::Affine3d _T;
        Eigen::Affine3d _Tcurr;
        Eigen::Vector6d _vel;
        Eigen::Vector6d _acc;
        std::pair<double, double> _max_vel, _max_acc;
        State _state;
        int _trj_segment;
    };

    LockFreeQueue<DataToClient, 1024> _to_cli_queue;
    DataToClient _cli_data, _rt_data;

    CartesianTask::Ptr _task_impl;

    // TaskDescription interface
public:
    void update(double time, double period) override;
};

} }

#endif // CARTESIANRT_H
