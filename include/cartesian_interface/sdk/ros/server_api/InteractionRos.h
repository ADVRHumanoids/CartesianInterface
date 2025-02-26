#ifndef INTERACTIONROS_SERVERAPI_H
#define INTERACTIONROS_SERVERAPI_H

#include "rclcpp_action/rclcpp_action.hpp"
#include <geometry_msgs/msg/wrench_stamped.hpp>

#include <cartesian_interface/sdk/ros/server_api/CartesianRos.h>

#include <cartesian_interface/problem/Interaction.h>

#include <cartesian_interface/action/reach_cartesian_impedance.hpp>
#include <cartesian_interface/srv/get_interaction_task_info.hpp>
#include <cartesian_interface/srv/get_impedance.hpp>
#include <cartesian_interface/srv/set_impedance.hpp>
#include <cartesian_interface/srv/set_impedance_ref_link.hpp>
#include <cartesian_interface/srv/get_force_limits.hpp>
#include <cartesian_interface/srv/set_force_limits.hpp>

#include <cartesian_interface/msg/interaction_task_info.hpp>

namespace XBot { namespace Cartesian {

using cartesian_interface::action::ReachCartesianImpedance;
using namespace cartesian_interface::msg;
using namespace cartesian_interface::srv;
using namespace geometry_msgs::msg;	

namespace ServerApi
{
	class InteractionRos;
}

/* fi: sorry for the ugly name...
 *
 * The acronymous stands for: ReachCartesianImpedanceActionManager
 */

class RCIAManager
{

public:

    RCIAManager(rclcpp::Node::SharedPtr node,
                InteractionTask::Ptr task);
	
	~RCIAManager() = default;
	
    void run();

private:

    using ActionServer = rclcpp_action::Server<ReachCartesianImpedance>;
    using ActionServerPtr = std::shared_ptr<ActionServer>;
    using GoalHandle = rclcpp_action::ServerGoalHandle<ReachCartesianImpedance>;

    enum class ReachActionState { IDLE, ACCEPTED, RUNNING, COMPLETED };

    void run_state_idle      ();
    void run_state_accepted  ();
    void run_state_running   ();
    void run_state_completed ();

    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &uuid,
                                            std::shared_ptr<const ReachCartesianImpedance::Goal> goal);

    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandle> goal_handle);

    void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle);

    rclcpp::Node::SharedPtr _node;
    ActionServerPtr      _server;
    std::shared_ptr<GoalHandle> _goal_handle;
    std::shared_ptr<ReachCartesianImpedance::Feedback> _fb;
    InteractionTask::Ptr _task  ;
    ReachActionState     _state ;
    std::string          _name  ;

};

class ServerApi::InteractionRos : public ServerApi::CartesianRos
{

public:

    InteractionRos(InteractionTask::Ptr task   ,
                   RosContext::Ptr      context);

    virtual void run(rclcpp::Time time) override;

private:

    void on_fref_recv(WrenchStamped::ConstSharedPtr msg);
	
	bool get_task_info_cb(GetInteractionTaskInfo::Request::ConstSharedPtr  req,
                          GetInteractionTaskInfo::Response::SharedPtr res);

    bool get_impedance_cb(GetImpedance::Request::ConstSharedPtr  req,
                          GetImpedance::Response::SharedPtr res);

    bool set_impedance_cb(SetImpedance::Request::ConstSharedPtr req,
                          SetImpedance::Response::SharedPtr res);
    
    bool get_force_limits_cb(GetForceLimits::Request::ConstSharedPtr  req,
                             GetForceLimits::Response::SharedPtr res);
    
    bool set_force_limits_cb(SetForceLimits::Request::ConstSharedPtr  req,
                             SetForceLimits::Response::SharedPtr res);

    bool set_impedance_ref_link_cb(SetImpedanceRefLink::Request::ConstSharedPtr req,
                                   SetImpedanceRefLink::Response::SharedPtr res);

    void publish_task_info();

    rclcpp::Subscription<WrenchStamped>::SharedPtr                                _fref_sub;
    rclcpp::Publisher<WrenchStamped>::SharedPtr                        _fref_pub;
    rclcpp::Publisher<CartesianImpedance>::SharedPtr                   _impd_pub;
    rclcpp::Publisher<InteractionTaskInfo>::SharedPtr                  _task_info_pub;
	
    rclcpp::ServiceBase::SharedPtr _get_info_srv, _get_impedance_srv, _set_impedance_srv;
    rclcpp::ServiceBase::SharedPtr _set_impedance_ref_link_srv, _get_force_limits_srv, _set_force_limits_srv;

    InteractionTask::Ptr         _ci_inter;
	
	std::unique_ptr<RCIAManager> _action  ;


};

} }


#endif // INTERACTIONROS_H
