#ifndef CARTESIANROS_H
#define CARTESIANROS_H

#include <cartesian_interface/problem/Cartesian.h>
#include "TaskRos.h"

#include <cartesian_interface/GetCartesianTaskInfo.h>
#include <cartesian_interface/CartesianTaskInfo.h>

#include <actionlib/client/simple_action_client.h>
#include <cartesian_interface/ReachPoseAction.h>

namespace XBot { namespace Cartesian {

namespace ClientApi
{
class CartesianRos;
}

class ClientApi::CartesianRos : virtual public CartesianTask,
        public ClientApi::TaskRos
{



    // CartesianTask interface
public:

    CARTESIO_DECLARE_SMART_PTR(CartesianRos)

    CartesianRos(std::string name,
                 ros::NodeHandle nh);

    bool validate() override;
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
    bool getPoseTarget(Eigen::Affine3d & base_T_ref) const override;
    bool setPoseTarget(const Eigen::Affine3d & base_T_ref, double time) override;
    int getCurrentSegmentId() const override;
    bool setWayPoints(const Trajectory::WayPointVector & way_points) override;
    void abort() override;

    bool waitReachCompleted(double timeout);

    bool setWayPoints(const Trajectory::WayPointVector& way_points,
                      bool incremental
                      );

private:

    typedef cartesian_interface::ReachPoseAction ActionType;
    typedef actionlib::SimpleActionClient<ActionType> ActionClient;

    cartesian_interface::GetCartesianTaskInfoResponse get_task_info() const;

    ros::Publisher _pose_ref_pub;
    ros::Publisher _vel_ref_pub;
    ros::Subscriber _pose_ref_sub;
    ros::Subscriber _vel_ref_sub;
    ros::ServiceClient _set_safety_lims_cli;
    ros::ServiceClient _set_base_link_cli;
    ros::ServiceClient _set_ctrl_mode_cli;
    mutable ros::ServiceClient _cart_info_cli;

    bool _Tref_recv, _vref_recv;
    Eigen::Affine3d _Tref;
    Eigen::Vector6d _vref;

    ActionClient _action_cli;
    int _current_segment_idx;

    mutable std::string _base_link, _distal_link;

    cartesian_interface::CartesianTaskInfo _info;

    void on_reach_feedback_recv(const cartesian_interface::ReachPoseFeedbackConstPtr& feedback);

    void on_action_active();

    void on_action_done(const actionlib::SimpleClientGoalState& state,
                        const cartesian_interface::ReachPoseResultConstPtr& result);


};

} }

#endif // CARTESIANROS_H
