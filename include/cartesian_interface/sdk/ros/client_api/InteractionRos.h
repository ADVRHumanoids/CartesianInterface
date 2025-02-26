#ifndef INTERACTIONROS__CLIENTAPI_H
#define INTERACTIONROS__CLIENTAPI_H

#include <cartesian_interface/problem/Interaction.h>
#include <cartesian_interface/sdk/ros/client_api/CartesianRos.h>

#include <cartesian_interface/action/reach_cartesian_impedance.hpp>
#include <cartesian_interface/srv/get_interaction_task_info.hpp>
#include <cartesian_interface/srv/get_impedance.hpp>
#include <cartesian_interface/srv/set_impedance.hpp>

#include <cartesian_interface/msg/interaction_task_info.hpp>
#include <cartesian_interface/msg/cartesian_impedance_timed.hpp>

#include <cartesian_interface/srv/set_impedance_ref_link.hpp>

#include <cartesian_interface/srv/get_force_limits.hpp>
#include <cartesian_interface/srv/set_force_limits.hpp>

#include <rclcpp_action/rclcpp_action.hpp>

namespace XBot { namespace Cartesian {

namespace ClientApi
{
class InteractionRos;
}

using cartesian_interface::action::ReachCartesianImpedance;

using cartesian_interface::msg::CartesianImpedanceTimed;
using cartesian_interface::msg::InteractionTaskInfo;

using cartesian_interface::srv::GetInteractionTaskInfo;
using cartesian_interface::srv::SetImpedanceRefLink;
using cartesian_interface::srv::GetForceLimits;
using cartesian_interface::srv::SetForceLimits;
using cartesian_interface::srv::GetImpedance;
using cartesian_interface::srv::SetImpedance;

class ClientApi::InteractionRos : virtual public InteractionTask,
        public ClientApi::CartesianRos
{

public:

    CARTESIO_DECLARE_SMART_PTR(InteractionRos)

    InteractionRos(std::string name,
				   rclcpp::Node::SharedPtr node);

    const Impedance & getImpedance ();
	
	const Eigen::Vector6d& getForceReference () const override;
    void getForceLimits (Eigen::Vector6d& fmax) const override;

    bool setImpedance (const Impedance & impedance) override;
	
	void setForceReference (const Eigen::Vector6d& f) override;
    bool setForceLimits (const Eigen::Vector6d& fmax) override;
	
	bool  waitTransitionCompleted (double timeout);
	void  abortStiffnessTransition () override;
	bool  setStiffnessTransition (const Interpolator<Eigen::Matrix6d>::WayPointVector & way_points) override;
	State getStiffnessState () const override;

    const std::string& getImpedanceRefLink() const;
    bool setImpedanceRefLink(const std::string& new_impedance_ref_link);
    

private:

    // typedef cartesian_interface::ReachCartesianImpedanceAction ActionType;
    // typedef actionlib::SimpleActionClient<ActionType> ActionClient;

    using ActionClient = rclcpp_action::Client<ReachCartesianImpedance>;
    using ActionGoalHandle = rclcpp_action::ClientGoalHandle<ReachCartesianImpedance>;

    ActionClient::SharedPtr _action_cli;
    ActionGoalHandle::SharedPtr _action_goal_handle;
    std::shared_future<ActionGoalHandle::SharedPtr> _action_future;
	
    rclcpp::Client<GetInteractionTaskInfo>::SharedPtr _interaction_info_cli;
    rclcpp::Client<SetImpedanceRefLink>::SharedPtr _set_impedance_ref_link_cli;
    rclcpp::Client<GetForceLimits>::SharedPtr _get_force_limits_cli;
    rclcpp::Client<SetForceLimits>::SharedPtr _set_force_limits_cli;
	rclcpp::Client<GetImpedance>::SharedPtr _get_impedance_cli;
    rclcpp::Client<SetImpedance>::SharedPtr _set_impedance_cli;

    rclcpp::Subscription<InteractionTaskInfo>::SharedPtr _task_info_sub;

    mutable std::string _impedance_ref_link;
	
	InteractionTaskInfo _info;
	
	GetInteractionTaskInfo::Response::SharedPtr get_task_info() const;
    
	void on_action_feedback(
        ActionGoalHandle::SharedPtr goal_handle,
        const std::shared_ptr<const ReachCartesianImpedance::Feedback> feedback);

    void on_action_active(ActionGoalHandle::SharedPtr goal_handle);

    void on_action_done(const ActionGoalHandle::WrappedResult& result);

    void on_task_info_recv(InteractionTaskInfo::ConstSharedPtr msg);
	
	Impedance _impedance;
	Eigen::Vector6d _f;

};

} }

#endif // INTERACTIONROS__CLIENTAPI_H
