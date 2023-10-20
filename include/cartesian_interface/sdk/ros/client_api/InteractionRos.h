#ifndef INTERACTIONROS__CLIENTAPI_H
#define INTERACTIONROS__CLIENTAPI_H

#include <cartesian_interface/problem/Interaction.h>
#include <cartesian_interface/sdk/ros/client_api/CartesianRos.h>

#include <actionlib/client/simple_action_client.h>
#include <cartesian_interface/ReachCartesianImpedanceAction.h>
#include <cartesian_interface/GetInteractionTaskInfo.h>
#include <cartesian_interface/GetImpedance.h>
#include <cartesian_interface/SetImpedance.h>
#include <cartesian_interface/InteractionTaskInfo.h>

#include <cartesian_interface/SetImpedanceRefLink.h>

#include <cartesian_interface/GetForceLimits.h>
#include <cartesian_interface/SetForceLimits.h>


namespace XBot { namespace Cartesian {

namespace ClientApi
{
class InteractionRos;
}

class ClientApi::InteractionRos : virtual public InteractionTask,
        public ClientApi::CartesianRos
{

public:

    CARTESIO_DECLARE_SMART_PTR(InteractionRos)

    InteractionRos(std::string name,
				   ros::NodeHandle nh);

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

    typedef cartesian_interface::ReachCartesianImpedanceAction ActionType;
    typedef actionlib::SimpleActionClient<ActionType> ActionClient;

    ActionClient _action_cli;
	
	mutable ros::ServiceClient _get_impedance_cli;
    mutable ros::ServiceClient _set_impedance_cli;
    mutable ros::ServiceClient _interaction_info_cli;
    ros::ServiceClient _set_impedance_ref_link_cli;
    mutable ros::ServiceClient _get_force_limits_cli;
    mutable ros::ServiceClient _set_force_limits_cli;

    ros::Subscriber _task_info_sub;

    mutable std::string _impedance_ref_link;
	
	cartesian_interface::InteractionTaskInfo _info;
	
	cartesian_interface::GetInteractionTaskInfoResponse get_task_info() const;
    
	void on_action_feedback(const cartesian_interface::ReachCartesianImpedanceFeedbackConstPtr& feedback);

    void on_action_active();

    void on_action_done(const actionlib::SimpleClientGoalState& state,
                        const cartesian_interface::ReachCartesianImpedanceResultConstPtr& result);

    void on_task_info_recv(cartesian_interface::InteractionTaskInfoConstPtr msg);
	
	Impedance _impedance;
	Eigen::Vector6d _f;

};

} }

#endif // INTERACTIONROS__CLIENTAPI_H
