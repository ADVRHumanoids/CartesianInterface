#ifndef CARTESIANROS__CLIENTAPI_H
#define CARTESIANROS__CLIENTAPI_H

#include <cartesian_interface/problem/Cartesian.h>
#include <cartesian_interface/sdk/ros/client_api/TaskRos.h>

#include <cartesian_interface/srv/get_cartesian_task_info.hpp>
#include <cartesian_interface/msg/cartesian_task_info.hpp>

#include <cartesian_interface/srv/set_base_link.hpp>
#include <cartesian_interface/srv/set_control_mode.hpp>
#include <cartesian_interface/srv/set_safety_limits.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

#include <rclcpp_action/rclcpp_action.hpp>
#include <cartesian_interface/action/reach_pose.hpp>

namespace XBot { namespace Cartesian {

namespace ClientApi
{
class CartesianRos;
}

using cartesian_interface::action::ReachPose;
using cartesian_interface::srv::GetCartesianTaskInfo;
using cartesian_interface::msg::CartesianTaskInfo;
using cartesian_interface::srv::SetBaseLink;
using cartesian_interface::srv::SetControlMode;
using cartesian_interface::srv::SetSafetyLimits;
using geometry_msgs::msg::PoseStamped;
using geometry_msgs::msg::TwistStamped;

class ClientApi::CartesianRos : virtual public CartesianTask,
        public ClientApi::TaskRos
{

public:

    CARTESIO_DECLARE_SMART_PTR(CartesianRos)

    CartesianRos(std::string name,
                 rclcpp::Node::SharedPtr node);

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
    bool setAccelerationReference(const Eigen::Vector6d & base_acc_ref) override;
    bool setVelocityReference(const Eigen::Vector6d& base_vel_ref,
                              const std::string& base_frame);
    bool getPoseTarget(Eigen::Affine3d & base_T_ref) const override;
    bool setPoseTarget(const Eigen::Affine3d & base_T_ref, double time) override;
    int getCurrentSegmentId() const override;
    bool setWayPoints(const Trajectory::WayPointVector & way_points) override;
    void abort() override;
    void registerObserver(TaskObserver::WeakPtr obs) override;

    bool waitReachCompleted(double timeout);

    bool setWayPoints(const Trajectory::WayPointVector& way_points,
                      bool incremental
                      );
protected:

    void notifyTaskChanged(const std::string & message) override;

private:

    GetCartesianTaskInfo::Response get_task_info() const;

    rclcpp::Publisher<PoseStamped>::SharedPtr _pose_ref_pub;
    rclcpp::Publisher<TwistStamped>::SharedPtr _vel_ref_pub;
    rclcpp::Subscription<PoseStamped>::SharedPtr _pose_ref_sub;
    rclcpp::Subscription<TwistStamped>::SharedPtr _vel_ref_sub;
    rclcpp::Subscription<CartesianTaskInfo>::SharedPtr _task_info_sub;
    rclcpp::Client<SetSafetyLimits>::SharedPtr _set_safety_lims_cli;
    rclcpp::Client<SetBaseLink>::SharedPtr _set_base_link_cli;
    rclcpp::Client<SetControlMode>::SharedPtr _set_ctrl_mode_cli;
    rclcpp::Client<GetCartesianTaskInfo>::SharedPtr _cart_info_cli;

    bool _Tref_recv, _vref_recv;
    Eigen::Affine3d _Tref;
    Eigen::Vector6d _vref;
    CartesianTaskInfo _info;

    rclcpp_action::Client<ReachPose>::SharedPtr _action_cli;
    int _current_segment_idx;

    std::list<CartesianTaskObserver::WeakPtr> _observers;

    mutable std::string _base_link, _distal_link;

    typedef rclcpp_action::ClientGoalHandle<ReachPose> ActionGoalHandle;

    void on_action_active(ActionGoalHandle::SharedPtr goal_handle);

    void on_reach_feedback_recv(
        ActionGoalHandle::SharedPtr goal_handle,
        const std::shared_ptr<const ReachPose::Feedback> fb
        );

    void on_action_done(const ActionGoalHandle::WrappedResult& result);

    void on_task_info_recv(CartesianTaskInfo::ConstSharedPtr msg);

    std::shared_future<ActionGoalHandle::SharedPtr> _action_future;

    ActionGoalHandle::SharedPtr _action_goal_handle;

};

} }

#endif // CARTESIANROS_H
