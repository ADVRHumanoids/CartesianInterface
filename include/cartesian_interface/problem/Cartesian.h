#ifndef __XBOT_CARTESIAN_PROBLEM_CARTESIAN_H__
#define __XBOT_CARTESIAN_PROBLEM_CARTESIAN_H__

#include <cartesian_interface/problem/Task.h>

#include <cartesian_interface/Enum.h>
#include <cartesian_interface/Context.h>
#include <cartesian_interface/trajectory/Trajectory.h>

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


    /**
     * @brief getter for the task base link
     */
    virtual const std::string& getBaseLink() const = 0;

    /**
     * @brief setBaseLink changes the task base link with the provided one
     * @param new_base_link must belong to the model kinematic tree
     * @return true if the new base link is valid and the operation was successful
     */
    virtual bool setBaseLink(const std::string& new_base_link) = 0;

    /**
     * @brief getter for the task distal link
     */
    virtual const std::string& getDistalLink() const = 0;

    /**
     * @brief getter for the task current control mode (i.e. Position, or Velocity)
     */
    virtual ControlType getControlMode() const = 0;

    /**
     * @brief setControlMode changes the current control mode
     * @return true if the operation was successful
     */
    virtual bool setControlMode(const ControlType & value) = 0;

    /**
     * @brief getter for the current task state, which indicates if the task
     * is currently performing a reaching motion, or if it is available for
     * online control
     */
    virtual State getTaskState() const = 0;

    /**
     * @brief isSubtaskLocal returns true if subtasks extracted from this task
     * should be in local (distal link) coordinates, and false otherwise (base link
     * coordinates)
     */
    virtual bool isSubtaskLocal() const = 0;

    /**
     * @brief getCurrentPose returns the current pose value of the task, i.e.
     * the pose of distal link relative to base link at the current solution
     * (i.e. the model state)
     * @param base_T_ee is filled in by the function
     * @return true on success
     */
    virtual bool getCurrentPose(Eigen::Affine3d& base_T_ee) const = 0; // ?

    /**
     * @brief getPoseReference returns the current desired value for the task pose,
     * and optionally velocity and acceleration
     * @param base_T_ref reference to desired pose
     * @param base_vel_ref pointer to desired velocity
     * @param base_acc_ref pointer to desired acceleration
     * @return true on success
     */
    virtual bool getPoseReference(Eigen::Affine3d& base_T_ref,
                                  Eigen::Vector6d * base_vel_ref = nullptr,
                                  Eigen::Vector6d * base_acc_ref = nullptr) const = 0;

    /**
     * @brief getPoseReferenceRaw returns the current desired value for the task pose,
     * and optionally velocity and acceleration. The returned value is BEFORE any filtering
     * or online trajectory generation being performed by the task. The references which are
     * actually fed to the controller are instead given by getReferencePose.
     * @param base_T_ref reference to desired pose
     * @param base_vel_ref pointer to desired velocity
     * @param base_acc_ref pointer to desired acceleration
     * @return true on success
     */
    virtual bool getPoseReferenceRaw(Eigen::Affine3d& base_T_ref,
                                     Eigen::Vector6d * base_vel_ref = nullptr,
                                     Eigen::Vector6d * base_acc_ref = nullptr) const = 0;

    /**
     * @brief setPoseReference sets a new pose reference for the controller.
     * If enableOnlineTrajectoryGeneration() was called, the provided reference is
     * saturated according to the active velocity and acceleration limits.
     * @param base_T_ref the new reference
     * @return true on success
     */
    virtual bool setPoseReference(const Eigen::Affine3d& base_T_ref) = 0;

    /**
     * @brief setPoseReferenceRaw does the same as setPoseReference, but
     * bypasses any trajectory generation. It is therefore less safe to use.
     */
    virtual bool setPoseReferenceRaw(const Eigen::Affine3d& base_T_ref) = 0;

    /**
     * @brief setVelocityReference sets a new desired velocity for the task.
     * @return
     */
    virtual bool setVelocityReference(const Eigen::Vector6d& base_vel_ref) = 0;

    /**
     * @brief setAccelerationReference sets a new desired acceleration for the task.
     * @return
     */
    virtual bool setAccelerationReference(const Eigen::Vector6d& base_acc_ref) = 0;

    /**
     * @brief getPoseTarget returns the current of the reaching motion being
     * performed by the task, if the task state is Reaching.
     * @return  true if the task state is Reaching, false otherwise
     */
    virtual bool getPoseTarget(Eigen::Affine3d& base_T_ref) const = 0;

    /**
     * @brief setPoseTarget commands a reaching motion to the given target
     * @param base_T_ref the pose to be reached
     * @param time the time (in seconds) to reach the task
     * @return true on success, i.e. the task state was 'Online`
     */
    virtual bool setPoseTarget(const Eigen::Affine3d& base_T_ref,
                               double time) = 0;

    /**
     * @brief setWayPoints commands a reaching motion through the specified list
     * of waypoints
     * @param way_points a std::vector of WayPoints, each of which specifies a frame,
     * and a time relative to the trajectory start
     * @return true on success, i.e. the task state was 'Online'
     */
    virtual bool setWayPoints(const Trajectory::WayPointVector& way_points) = 0;

    /**
     * @brief getCurrentSegmentId returns the trajectory segment id currently being performed,
     * or -1 if the task state is 'Online'
     */
    virtual int getCurrentSegmentId() const = 0;

    /**
     * @brief abort interrupts any reaching motion
     */
    virtual void abort() = 0;

    /**
     * @brief getVelocityLimits return the currently active velocity limits for linear and angular motions
     */
    virtual void getVelocityLimits(double& max_vel_lin,
                                   double& max_vel_ang) const = 0;

    /**
     * @brief getAccelerationLimits return the currently active acceleration limits for linear and angular motions
     */
    virtual void getAccelerationLimits(double& max_acc_lin,
                                       double& max_acc_ang) const = 0;

    /**
     * @brief setVelocityLimits sets new velocity limits for linear and angular motions
     */
    virtual void setVelocityLimits(double max_vel_lin,
                                   double max_vel_ang) = 0;

    /**
     * @brief setAccelerationLimits sets new acceleration limits for linear and angular motions
     */
    virtual void setAccelerationLimits(double max_acc_lin,
                                       double max_acc_ang) = 0;

    virtual bool getNormalizedFlag() const = 0;
    virtual bool getMareyGainFlag() const = 0;
    virtual double getMareyGainE0() const = 0;
    virtual double getMareyGainE1() const = 0;
    virtual double getRegularization() const = 0;

};

} }


#endif
