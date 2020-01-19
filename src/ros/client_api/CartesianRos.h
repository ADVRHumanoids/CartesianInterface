#ifndef CARTESIANROS_H
#define CARTESIANROS_H

#include <cartesian_interface/problem/Cartesian.h>
#include "TaskRos.h"

namespace XBot { namespace Cartesian {

class CartesianRos : virtual public CartesianTask,
                     public TaskRos
{

public:

    CartesianRos(std::string name);

    void getVelocityLimits(double & max_vel_lin, double & max_vel_ang) const override;
    void getAccelerationLimits(double & max_acc_lin, double & max_acc_ang) const override;
    void setVelocityLimits(double max_vel_lin, double max_vel_ang) override;
    void setAccelerationLimits(double max_acc_lin, double max_acc_ang) override;
    void enableOtg() override;
    State getTaskState() const override;
    const std::string & getBaseLink() const override;
    bool setBaseLink(const std::string & new_base_link) override;
    const std::string & getDistalLink() const override;
    bool getCurrentPose(Eigen::Affine3d & base_T_ee) const override;
    bool getPoseReference(Eigen::Affine3d & base_T_ref,
                          Eigen::Vector6d * base_vel_ref,
                          Eigen::Vector6d * base_acc_ref) const override;
    bool getPoseReferenceRaw(Eigen::Affine3d & base_T_ref,
                             Eigen::Vector6d * base_vel_ref,
                             Eigen::Vector6d * base_acc_ref) const override;
    bool setPoseReference(const Eigen::Affine3d & base_T_ref) override;
    bool setPoseReferenceRaw(const Eigen::Affine3d & base_T_ref) override;
    bool setVelocityReference(const Eigen::Vector6d & base_vel_ref) override;
    bool getPoseTarget(Eigen::Affine3d & base_T_ref) const override;
    bool setPoseTarget(const Eigen::Affine3d & base_T_ref, double time) override;
    int getCurrentSegmentId() const override;
    bool setWayPoints(const Trajectory::WayPointVector & way_points) override;
    void abort() override;

    virtual ~CartesianRos();;

private:
    Eigen::Affine3d get_current_pose() const override;
};

} }

#endif // CARTESIANROS_H
