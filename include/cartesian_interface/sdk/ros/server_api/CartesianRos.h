#ifndef CARTESIANROSAPI_SERVERAPI_H
#define CARTESIANROSAPI_SERVERAPI_H

#include <cartesian_interface/sdk/ros/server_api/TaskRos.h>

#include <cartesian_interface/problem/Cartesian.h>

#include "rclcpp_action/rclcpp_action.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

#include <cartesian_interface/action/reach_pose.hpp>

#include <cartesian_interface/srv/get_cartesian_task_info.hpp>
#include <cartesian_interface/srv/set_base_link.hpp>
#include <cartesian_interface/srv/set_control_mode.hpp>
#include <cartesian_interface/srv/set_safety_limits.hpp>

#include <cartesian_interface/msg/cartesian_task_info.hpp>


namespace XBot { namespace Cartesian {

using namespace cartesian_interface::msg;
using namespace cartesian_interface::srv;
using namespace cartesian_interface::action;
using namespace geometry_msgs::msg;

class ReachActionManager
{

public:

    ReachActionManager(rclcpp::Node::SharedPtr node,
                       std::string task_name,
                       CartesianTask::Ptr task);

    void run();

    ~ReachActionManager();

private:

    typedef rclcpp_action::Server<ReachPose> ActionServer;
    typedef rclcpp_action::ServerGoalHandle<ReachPose> GoalHandle;
    typedef std::shared_ptr<ActionServer> ActionServerPtr;

    enum class ReachActionState { IDLE, ACCEPTED, RUNNING, COMPLETED };

    void run_state_idle();
    void run_state_accepted();
    void run_state_running();
    void run_state_completed();

    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &uuid,
                                            std::shared_ptr<const ReachPose::Goal> goal);

    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandle> goal_handle);

    void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle);

    rclcpp::Node::SharedPtr _node;
    ActionServerPtr _action_server;
    std::shared_ptr<GoalHandle> _goal_handle;
    std::shared_ptr<ReachPose::Feedback> _fb;
    CartesianTask::Ptr _task;
    ReachActionState _state;
    std::string _name;

};

namespace ServerApi
{
    class CartesianRos;
}

class ServerApi::CartesianRos : public ServerApi::TaskRos,
        public virtual CartesianTaskObserver
{

public:

    CartesianRos(CartesianTask::Ptr task,
                 RosContext::Ptr context);

    bool onBaseLinkChanged() override;
    bool onControlModeChanged() override;

    virtual void run(rclcpp::Time time) override;

protected:

private:

    void publish_ref(rclcpp::Time time);

    void publish_task_info();

    void online_position_reference_cb(PoseStamped::ConstSharedPtr msg);

    void online_velocity_reference_cb(TwistStamped::ConstSharedPtr msg);

    bool get_task_info_cb(GetCartesianTaskInfo::Request::ConstSharedPtr req,
                          GetCartesianTaskInfo::Response::SharedPtr res);

    bool set_base_link_cb(SetBaseLink::Request::ConstSharedPtr req,
                          SetBaseLink::Response::SharedPtr res);

    bool set_control_mode_cb(SetControlMode::Request::ConstSharedPtr req,
                             SetControlMode::Response::SharedPtr res);

    bool set_safety_lims_cb(SetSafetyLimits::Request::ConstSharedPtr req,
                            SetSafetyLimits::Response::SharedPtr res);



    rclcpp::Publisher<PoseStamped>::SharedPtr _pose_ref_pub;
    rclcpp::Publisher<TwistStamped>::SharedPtr _vel_ref_pub;
    rclcpp::Publisher<TwistStamped>::SharedPtr _acc_ref_pub;
    rclcpp::Publisher<CartesianTaskInfo>::SharedPtr _task_info_pub;
    rclcpp::SubscriptionBase::SharedPtr _pose_ref_sub, _vel_ref_sub;
    rclcpp::ServiceBase::SharedPtr _get_info_srv, _set_base_link_srv,
        _set_ctrl_srv, _set_safety_srv;

    CartesianTask::Ptr _cart;

    std::unique_ptr<ReachActionManager> _reach_action_manager;

};

} }

#endif // CARTESIANROSAPI_H
