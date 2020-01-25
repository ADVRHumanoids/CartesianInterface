#ifndef __XBOT_CARTESIAN_PROBLEM_CARTESIAN_H__
#define __XBOT_CARTESIAN_PROBLEM_CARTESIAN_H__

#include <cartesian_interface/problem/Task.h>

#include <cartesian_interface/Enum.h>
#include <cartesian_interface/Context.h>
#include <cartesian_interface/trajectory/Trajectory.h>
#include <ReflexxesTypeII/Wrappers/TrajectoryGenerator.h>

namespace XBot { namespace Cartesian {

class CartesianTaskObserver : public virtual TaskObserver
{

public:

    typedef std::weak_ptr<CartesianTaskObserver> WeakPtr;

    virtual bool onBaseLinkChanged();
    virtual bool onControlModeChanged();
    virtual bool onSafetyLimitsChanged();

    virtual ~CartesianTaskObserver() = default;
};

/**
 * @brief Abstract interface of a cartesian task
 */
class CartesianTask : public virtual TaskDescription
{

public:

    CARTESIO_DECLARE_SMART_PTR(CartesianTask)

    virtual void enableOnlineTrajectoryGeneration() = 0;

    /* Parameters */
    virtual bool isSubtaskLocal() const = 0;

    /* Safety limits */
    virtual void getVelocityLimits(double& max_vel_lin,
                                   double& max_vel_ang) const = 0;

    virtual void getAccelerationLimits(double& max_acc_lin,
                                       double& max_acc_ang) const = 0;

    virtual void setVelocityLimits(double max_vel_lin,
                                   double max_vel_ang) = 0;

    virtual void setAccelerationLimits(double max_acc_lin,
                                       double max_acc_ang) = 0;

    virtual State getTaskState() const = 0;

    virtual const std::string& getBaseLink() const = 0;
    virtual bool setBaseLink(const std::string& new_base_link) = 0;

    virtual const std::string& getDistalLink() const = 0;

    virtual ControlType getControlMode() const = 0;
    virtual bool setControlMode(const ControlType & value) = 0;

    virtual bool getCurrentPose(Eigen::Affine3d& base_T_ee) const = 0; // ?

    virtual bool getPoseReference(Eigen::Affine3d& base_T_ref,
                                  Eigen::Vector6d * base_vel_ref = nullptr,
                                  Eigen::Vector6d * base_acc_ref = nullptr) const = 0;

    virtual bool getPoseReferenceRaw(Eigen::Affine3d& base_T_ref,
                                     Eigen::Vector6d * base_vel_ref = nullptr,
                                     Eigen::Vector6d * base_acc_ref = nullptr) const = 0;

    virtual bool setPoseReference(const Eigen::Affine3d& base_T_ref) = 0;

    virtual bool setPoseReferenceRaw(const Eigen::Affine3d& base_T_ref) = 0;

    virtual bool setVelocityReference(const Eigen::Vector6d& base_vel_ref) = 0;

    virtual bool getPoseTarget(Eigen::Affine3d& base_T_ref) const = 0;

    virtual bool setPoseTarget(const Eigen::Affine3d& base_T_ref,
                               double time) = 0;

    virtual int getCurrentSegmentId() const = 0;

    virtual bool setWayPoints(const Trajectory::WayPointVector& way_points) = 0;

    virtual void abort() = 0;

};

} }


#endif
