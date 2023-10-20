#ifndef INTERACTIONROS_SERVERAPI_H
#define INTERACTIONROS_SERVERAPI_H

#include <geometry_msgs/WrenchStamped.h>

#include <cartesian_interface/sdk/ros/server_api/CartesianRos.h>
#include <cartesian_interface/problem/Interaction.h>

#include <cartesian_interface/ReachCartesianImpedanceAction.h>
#include <cartesian_interface/GetInteractionTaskInfo.h>
#include <cartesian_interface/GetImpedance.h>
#include <cartesian_interface/SetImpedance.h>
#include <cartesian_interface/SetImpedanceRefLink.h>
#include <cartesian_interface/GetForceLimits.h>
#include <cartesian_interface/SetForceLimits.h>

#include <cartesian_interface/InteractionTaskInfo.h>

namespace XBot { namespace Cartesian {


using cartesian_interface::ReachCartesianImpedanceAction;
	
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

    RCIAManager(ros::NodeHandle      nh,
                InteractionTask::Ptr task);
	
	~RCIAManager() = default;
	
    void run();

private:

    typedef actionlib::SimpleActionServer<ReachCartesianImpedanceAction> ActionServer;
    typedef std::shared_ptr<ActionServer> ActionServerPtr;

    enum class ReachActionState { IDLE, ACCEPTED, RUNNING, COMPLETED };

    void run_state_idle      ();
    void run_state_accepted  ();
    void run_state_running   ();
    void run_state_completed ();

    ActionServerPtr      _server;
    InteractionTask::Ptr _task  ;
    ReachActionState     _state ;
    std::string          _name  ;

};

class ServerApi::InteractionRos : public ServerApi::CartesianRos
{

public:

    InteractionRos(InteractionTask::Ptr task   ,
                   RosContext::Ptr      context);

    virtual void run(ros::Time time) override;

private:
	
    void on_fref_recv(geometry_msgs::WrenchStampedConstPtr msg);
	
	bool get_task_info_cb(cartesian_interface::GetInteractionTaskInfoRequest&  req,
                          cartesian_interface::GetInteractionTaskInfoResponse& res);

    bool get_impedance_cb(cartesian_interface::GetImpedanceRequest&  req,
                          cartesian_interface::GetImpedanceResponse& res);

    bool set_impedance_cb(cartesian_interface::SetImpedanceRequest& req,
                          cartesian_interface::SetImpedanceResponse& res);
    
    bool get_force_limits_cb(cartesian_interface::GetForceLimitsRequest&  req,
                             cartesian_interface::GetForceLimitsResponse& res);
    
    bool set_force_limits_cb(cartesian_interface::SetForceLimitsRequest&  req,
                             cartesian_interface::SetForceLimitsResponse& res);
    
    ros::Subscriber              _fref_sub;
    ros::Publisher               _fref_pub, _impd_pub, _task_info_pub;
	
    ros::ServiceServer _get_info_srv, _get_impedance_srv, _set_impedance_srv;
    ros::ServiceServer _set_impedance_ref_link_srv, _get_force_limits_srv, _set_force_limits_srv;

    InteractionTask::Ptr         _ci_inter;
	
	std::unique_ptr<RCIAManager> _action  ;

    void publish_task_info();

    bool set_impedance_ref_link_cb(cartesian_interface::SetImpedanceRefLinkRequest& req,
                          cartesian_interface::SetImpedanceRefLinkResponse& res);

};

} }


#endif // INTERACTIONROS_H
