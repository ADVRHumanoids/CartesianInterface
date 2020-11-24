#ifndef INTERACTIONROS__CLIENTAPI_H
#define INTERACTIONROS__CLIENTAPI_H

#include <cartesian_interface/problem/Interaction.h>
#include <cartesian_interface/sdk/ros/client_api/CartesianRos.h>

#include <actionlib/client/simple_action_client.h>
#include <cartesian_interface/ReachCartesianImpedanceAction.h>
#include <cartesian_interface/GetInteractionTaskInfo.h>
#include <cartesian_interface/GetImpedance.h>
#include <cartesian_interface/InteractionTaskInfo.h>


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
    void getForceLimits (Eigen::Vector6d& fmin, Eigen::Vector6d& fmax) const override;

    void setImpedance (const Impedance & impedance) override;
	
	void setForceReference (const Eigen::Vector6d& f) override;
    bool setForceLimits (const Eigen::Vector6d& fmin, const Eigen::Vector6d& fmax) override;
	
	bool  waitTransitionCompleted (double timeout);
	void  abortStiffnessTransition () override;
	bool  setStiffnessTransition (const Interpolator<Eigen::Matrix6d>::WayPointVector & way_points) override;
	State getStiffnessState () const override;

private:

    typedef cartesian_interface::ReachCartesianImpedanceAction ActionType;
    typedef actionlib::SimpleActionClient<ActionType> ActionClient;

    ActionClient _action_cli;
	
	mutable ros::ServiceClient _get_impedance_cli;
    mutable ros::ServiceClient _interaction_info_cli;
	
	cartesian_interface::InteractionTaskInfo _info;
	
	cartesian_interface::GetInteractionTaskInfoResponse get_task_info() const;
    
	void on_action_feedback(const cartesian_interface::ReachCartesianImpedanceFeedbackConstPtr& feedback);

    void on_action_active();

    void on_action_done(const actionlib::SimpleClientGoalState& state,
                        const cartesian_interface::ReachCartesianImpedanceResultConstPtr& result);
	
	Impedance _impedance;
	Eigen::Vector6d _f;

};

} }

#endif // INTERACTIONROS__CLIENTAPI_H
