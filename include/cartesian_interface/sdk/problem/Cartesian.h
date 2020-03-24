#ifndef __XBOT_CARTESIAN_PROBLEM_CARTESIAN_IMPL_H__
#define __XBOT_CARTESIAN_PROBLEM_CARTESIAN_IMPL_H__

#include <cartesian_interface/sdk/problem/Task.h>
#include <cartesian_interface/problem/Cartesian.h>

#include <cartesian_interface/Enum.h>
#include <cartesian_interface/Context.h>
#include <cartesian_interface/trajectory/Trajectory.h>
#include <ReflexxesTypeII/Wrappers/TrajectoryGenerator.h>

namespace XBot { namespace Cartesian {

class CartesianTaskImpl :
        public TaskDescriptionImpl,
        public virtual CartesianTask
{

public:

    CARTESIO_DECLARE_SMART_PTR(CartesianTaskImpl)

    /* Constructors */
    CartesianTaskImpl() = default;

    CartesianTaskImpl(Context::ConstPtr context,
                      std::string name,
                      std::string distal_link,
                      std::string base_link = "world");

    CartesianTaskImpl(Context::ConstPtr context,
                      std::string name,
                      std::string type,
                      std::string distal_link,
                      std::string base_link);

    CartesianTaskImpl(YAML::Node node, Context::ConstPtr context);

    virtual bool validate() override;

    /* Parameters */
    bool isSubtaskLocal() const override;

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
    void enableOnlineTrajectoryGeneration() override;

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

    void update(double time, double period) override;

    void reset() override;

    virtual void log(MatLogger2::Ptr logger,
                     bool init_logger = false,
                     int buf_size = 1e5) override;

    virtual void registerObserver(TaskObserver::WeakPtr observer);

private:

    ControlType _ctrl_mode;

    std::list<CartesianTaskObserver::WeakPtr> _observers;

    /**
         * @brief The task controls the relative motion of distal_link w.r.t base_link
         */
    std::string _base_link, _distal_link;

    /**
         * @brief Set Cartesian task in local (body) frame
         */
    bool _is_body_jacobian;

    /**
         * @brief Parameter that weights orientation errors w.r.t. position errors.
         * For example, setting it to orientation_gain = 2.0 means that an error of
         * 2 rad is recovered in the same time as an error of 1 meter.
         */
    double _orientation_gain;

    /* Task implementation variables (TBD: pimpl) */

    typedef Eigen::Matrix<double, 7, 1> EigenVector7d;

    bool check_reach() const;
    void apply_otg();
    void reset_otg();
    Eigen::Affine3d get_pose_ref_otg() const;
    virtual Eigen::Affine3d get_current_pose() const;

    Eigen::Affine3d _T;
    Eigen::Vector6d _vel;
    Eigen::Vector6d _acc;

    EigenVector7d _otg_des, _otg_ref, _otg_vref;
    EigenVector7d _otg_maxvel, _otg_maxacc;

    State _state;
    double _vref_time_to_live;

    Trajectory::Ptr _trajectory;
    Reflexxes::Utils::TrajectoryGenerator::Ptr _otg;


};

} }


#endif
