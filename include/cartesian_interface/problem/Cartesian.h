#ifndef __XBOT_CARTESIAN_PROBLEM_CARTESIAN_H__
#define __XBOT_CARTESIAN_PROBLEM_CARTESIAN_H__

#include <cartesian_interface/problem/Task.h>

#include <cartesian_interface/Enum.h>
#include <cartesian_interface/Context.h>
#include <cartesian_interface/trajectory/Trajectory.h>
#include <ReflexxesTypeII/Wrappers/TrajectoryGenerator.h>

namespace XBot { namespace Cartesian {

class CartesianTaskObserver
{

public:

    typedef std::weak_ptr<CartesianTaskObserver> WeakPtr;

    virtual bool onBaseLinkChanged();
    virtual bool onControlModeChanged();
    virtual bool onSafetyLimitsChanged();

    virtual ~CartesianTaskObserver() = default;
};

/**
     * @brief Description of a cartesian task
     */
class CartesianTask : public TaskDescription {

public:

    typedef std::shared_ptr<CartesianTask> Ptr;
    typedef std::shared_ptr<const CartesianTask> ConstPtr;

    /* Constructors */
    CartesianTask() = default;

    CartesianTask(std::string name,
                  std::string distal_link,
                  std::string base_link = "world");

    CartesianTask(YAML::Node node, ModelInterface::ConstPtr model);

    /* Safety limits */
    virtual void getVelocityLimits(double& max_vel_lin,
                                   double& max_vel_ang) const;

    virtual void getAccelerationLimits(double& max_acc_lin,
                                       double& max_acc_ang) const;

    virtual void setVelocityLimits(double max_vel_lin,
                                   double max_vel_ang);

    virtual void setAccelerationLimits(double max_acc_lin,
                                       double max_acc_ang);

    /**
         * @brief enableOtg enables online trajectory generation,
         * which enforces safety limtis to the Cartesian reference signals
         */
    virtual void enableOtg();

    virtual State getTaskState() const;

    virtual const std::string& getBaseLink() const;
    virtual bool setBaseLink(const std::string& new_base_link);

    virtual const std::string& getDistalLink() const;

    ControlType getControlMode() const;
    bool setControlMode(const ControlType & value);

    virtual bool getCurrentPose(Eigen::Affine3d& base_T_ee) const; // ?

    virtual bool getPoseReference(Eigen::Affine3d& base_T_ref,
                                  Eigen::Vector6d * base_vel_ref = nullptr,
                                  Eigen::Vector6d * base_acc_ref = nullptr) const;

    virtual bool getPoseReferenceRaw(Eigen::Affine3d& base_T_ref,
                                     Eigen::Vector6d * base_vel_ref = nullptr,
                                     Eigen::Vector6d * base_acc_ref = nullptr) const;

    virtual bool setPoseReference(const Eigen::Affine3d& base_T_ref);

    virtual bool setPoseReferenceRaw(const Eigen::Affine3d& base_T_ref);

    virtual bool setVelocityReference(const Eigen::Vector6d& base_vel_ref);

    virtual bool getPoseTarget(Eigen::Affine3d& base_T_ref) const;

    virtual bool setPoseTarget(const Eigen::Affine3d& base_T_ref,
                               double time);

    virtual int getCurrentSegmentId() const;

    virtual bool setWayPoints(const Trajectory::WayPointVector& way_points);

    virtual void abort();

    void run(double time, double period) override;

    void reset() override;

    void registerObserver(CartesianTaskObserver::WeakPtr observer);

private:

    ControlType ctrl_mode;

    std::list<CartesianTaskObserver::WeakPtr> _observers;

    /**
         * @brief The task controls the relative motion of distal_link w.r.t base_link
         */
    std::string base_link, distal_link;

    /**
         * @brief Set Cartesian task in local (body) frame
         */
    bool is_body_jacobian;

    /**
         * @brief Parameter that weights orientation errors w.r.t. position errors.
         * For example, setting it to orientation_gain = 2.0 means that an error of
         * 2 rad is recovered in the same time as an error of 1 meter.
         */
    double orientation_gain;

    /* Task implementation variables (TBD: pimpl) */

    typedef Eigen::Matrix<double, 7, 1> EigenVector7d;

    bool check_reach() const;
    void apply_otg();
    void reset_otg();
    Eigen::Affine3d get_pose_ref_otg() const;
    virtual Eigen::Affine3d get_current_pose() const;

    double current_time;

    Eigen::Affine3d T;
    Eigen::Vector6d vel;
    Eigen::Vector6d acc;

    EigenVector7d __otg_des, __otg_ref, __otg_vref;
    EigenVector7d __otg_maxvel, __otg_maxacc;

    ControlType control_type;
    State state;
    double vref_time_to_live;

    ModelInterface::ConstPtr model;

    Trajectory::Ptr trajectory;
    Reflexxes::Utils::TrajectoryGenerator::Ptr otg;

    Context ctx;
};

} }


#endif
