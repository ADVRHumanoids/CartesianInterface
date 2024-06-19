#include "fmt/format.h"
#include "ros/client_api/InteractionRos.h"

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <eigen_conversions/eigen_msg.h>

using namespace XBot::Cartesian;
using namespace XBot::Cartesian::ClientApi;
using namespace cartesian_interface;

InteractionRos::InteractionRos(std::string name,
							   ros::NodeHandle nh):
    CartesianRos(name, nh),
    _action_cli(nh, name + "/stiffness", false)
    // ,_Tref_recv(false), _vref_recv(false)
{
	_interaction_info_cli = _nh.serviceClient<GetInteractionTaskInfo>(name + "/get_interaction_task_properties");
    
	if(!_interaction_info_cli.waitForExistence(ros::Duration(1.0)) || !_interaction_info_cli.exists())
    {
        throw std::runtime_error(fmt::format("Non existent service '{}'",
                                             _interaction_info_cli.getService()));
    }
    
    _get_impedance_cli = _nh.serviceClient<GetImpedance>(name + "/get_impedance");
    
	if(!_get_impedance_cli.waitForExistence(ros::Duration(1.0)) || !_get_impedance_cli.exists())
    {
        throw std::runtime_error(fmt::format("Non existent service '{}'",
                                             _get_impedance_cli.getService()));
    }

    _set_impedance_cli = _nh.serviceClient<SetImpedance>(name + "/set_impedance");

    if(!_set_impedance_cli.waitForExistence(ros::Duration(1.0)) || !_set_impedance_cli.exists())
    {
        throw std::runtime_error(fmt::format("Non existent service '{}'",
                                             _set_impedance_cli.getService()));
    }

    _set_impedance_ref_link_cli = _nh.serviceClient<SetImpedanceRefLink>(name + "/set_impedance_ref_link");

    if(!_set_impedance_ref_link_cli.waitForExistence(ros::Duration(1.0)) || !_set_impedance_ref_link_cli.exists())
    {
        throw std::runtime_error(fmt::format("Non existent service '{}'",
                                             _set_impedance_ref_link_cli.getService()));
    }

    _get_force_limits_cli = _nh.serviceClient<GetForceLimits>(name + "/get_force_limits");
    
	if(!_get_force_limits_cli.waitForExistence(ros::Duration(1.0)) || !_get_force_limits_cli.exists())
    {
        throw std::runtime_error(fmt::format("Non existent service '{}'",
                                             _get_force_limits_cli.getService()));
    }

    _set_force_limits_cli = _nh.serviceClient<SetForceLimits>(name + "/set_force_limits");
    
	if(!_set_force_limits_cli.waitForExistence(ros::Duration(1.0)) || !_set_force_limits_cli.exists())
    {
        throw std::runtime_error(fmt::format("Non existent service '{}'",
                                             _set_force_limits_cli.getService()));
    }

    _task_info_sub = _nh.subscribe(name + "/interaction_task_properties", 10,
                                 &InteractionRos::on_task_info_recv, this);
        
    if(!_action_cli.isServerConnected())
    {
        /*throw std::runtime_error(fmt::format("Unable to reach action server '{}'",
                                             nh.resolveName(name + "/stiffness")));*/
    }
    
    _f.setZero();
}

GetInteractionTaskInfoResponse InteractionRos::get_task_info() const
{
    if(asyncMode())
    {
        GetInteractionTaskInfoResponse res;
        
		res.state = _info.state;
        res.state = _info.impedance_ref_link;
        
        return res;
    }

    cartesian_interface::GetInteractionTaskInfo srv;
    if(!_interaction_info_cli.call(srv))
    {
        throw std::runtime_error(fmt::format("Unable to call service '{}'",
                                             _interaction_info_cli.getService()));
    }

    return srv.response;
}

const Impedance & InteractionRos::getImpedance()
{
	cartesian_interface::GetImpedance srv;
    if(!_get_impedance_cli.call(srv))
    {
        throw std::runtime_error(fmt::format("Unable to call service '{}'",
                                             _get_impedance_cli.getService()));
    }
	
	// get current state for task (note: should it be getPoseReference instead?)
	Eigen::Vector3d temp1, temp2;
	Eigen::Vector6d temp3;
		
	tf::vectorMsgToEigen(srv.response.impedance.linear.stiffness,  temp1);
	tf::vectorMsgToEigen(srv.response.impedance.angular.stiffness, temp2);
			
	temp3.head(3) = temp1; temp3.tail(3) = temp2;
	
	_impedance.stiffness = temp3.asDiagonal();
	
	tf::vectorMsgToEigen(srv.response.impedance.linear.damping_ratio,  temp1);
	tf::vectorMsgToEigen(srv.response.impedance.angular.damping_ratio, temp2);
	
	temp3.head(3) = temp1; temp3.tail(3) = temp2;
	
	_impedance.damping = temp3.asDiagonal();
	
	_impedance.mass.setZero();
	
	return _impedance;
}

const Eigen::Vector6d& InteractionRos::getForceReference () const
{
	ROS_WARN("unsupported function: getForceReference");
	
	return _f;
}

bool InteractionRos::setImpedance (const Impedance & impedance)
{
    cartesian_interface::SetImpedance srv;

    tf::vectorEigenToMsg(impedance.stiffness.diagonal().head(3), srv.request.impedance.linear.stiffness);
    tf::vectorEigenToMsg(impedance.stiffness.diagonal().tail(3), srv.request.impedance.angular.stiffness);

    tf::vectorEigenToMsg(impedance.damping.diagonal().head(3), srv.request.impedance.linear.damping_ratio);
    tf::vectorEigenToMsg(impedance.damping.diagonal().tail(3), srv.request.impedance.angular.damping_ratio);

    if(!_set_impedance_cli.call(srv))
    {
        throw std::runtime_error(fmt::format("Unable to call service '{}'",
                                             _set_impedance_cli.getService()));
    }

    ROS_INFO("%s", srv.response.message.c_str());

    return srv.response.success;
}

void InteractionRos::setForceReference (const Eigen::Vector6d& f)
{
	ROS_WARN("unsupported function: setForceReference");
}

void InteractionRos::getForceLimits (Eigen::Vector6d& fmax) const
{
	cartesian_interface::GetForceLimits srv;
    if(!_get_force_limits_cli.call(srv))
    {
        throw std::runtime_error(fmt::format("Unable to call service '{}'",
                                             _get_force_limits_cli.getService()));
    }
	
	// get current state for task (note: should it be getPoseReference instead?)
	Eigen::Vector3d force, torque;
		
	tf::vectorMsgToEigen(srv.response.fmax.force, force);
	tf::vectorMsgToEigen(srv.response.fmax.torque, torque);
			
	fmax << force, torque;
}

bool InteractionRos::setForceLimits (const Eigen::Vector6d& fmax)
{
    SetForceLimits srv;
    tf::vectorEigenToMsg(fmax.head(3), srv.request.fmax.force);
    tf::vectorEigenToMsg(fmax.tail(3), srv.request.fmax.torque);

    if(!_set_force_limits_cli.call(srv))
    {
        throw std::runtime_error(fmt::format("Unable to call service '{}'",
                                             _set_force_limits_cli.getService()));
    }

    ROS_INFO("%s", srv.response.message.c_str());

    return srv.response.success;
}

State InteractionRos::getStiffnessState() const
{
    return StringToEnum<State>(get_task_info().state);
}

void InteractionRos::abortStiffnessTransition()
{
    _action_cli.cancelAllGoals();
}

bool InteractionRos::waitTransitionCompleted(double timeout)
{
    return _action_cli.waitForResult(ros::Duration(timeout));
}

bool InteractionRos::setStiffnessTransition(const Interpolator<Eigen::Matrix6d>::WayPointVector & way_points)
{
    cartesian_interface::ReachCartesianImpedanceGoal goal;
    
	Impedance impedance = getImpedance();
	
	for(const auto& wp : way_points)
    {
		cartesian_interface::CartesianImpedanceTimed cit;
		
		tf::vectorEigenToMsg (wp.value.diagonal().head(3), cit.impedance.linear.stiffness );
		tf::vectorEigenToMsg (wp.value.diagonal().tail(3), cit.impedance.angular.stiffness);
		
		tf::vectorEigenToMsg (impedance.damping.diagonal().head(3), cit.impedance.linear.damping_ratio );
		tf::vectorEigenToMsg (impedance.damping.diagonal().tail(3), cit.impedance.angular.damping_ratio);
		
		cit.time = wp.time;
		
		goal.target.push_back(cit);
    }

    _action_cli.sendGoal(goal,
                         boost::bind(&InteractionRos::on_action_done, this, _1, _2),
                         boost::bind(&InteractionRos::on_action_active, this),
                         boost::bind(&InteractionRos::on_action_feedback, this, _1));
    return true;
}

const std::string & InteractionRos::getImpedanceRefLink() const
{
    _impedance_ref_link = get_task_info().impedance_ref_link;
    return _impedance_ref_link;
}

bool InteractionRos::setImpedanceRefLink(const std::string & new_impedance_ref_link)
{
    SetImpedanceRefLink srv;
    srv.request.impedance_ref_link = new_impedance_ref_link;

    if(!_set_impedance_ref_link_cli.call(srv))
    {
        throw std::runtime_error(fmt::format("Unable to call service '{}'",
                                             _set_impedance_ref_link_cli.getService()));
    }

    ROS_INFO("%s", srv.response.message.c_str());

    return srv.response.success;
}

void InteractionRos::on_task_info_recv(InteractionTaskInfoConstPtr msg)
{
    _info = *msg;
}

void InteractionRos::on_action_feedback(const ReachCartesianImpedanceFeedbackConstPtr & feedback)
{
    
}

void InteractionRos::on_action_active()
{
    ROS_INFO("Reach action for task '%s' has become active",
             getName().c_str());
}

void InteractionRos::on_action_done(const actionlib::SimpleClientGoalState & state,
                                  const ReachCartesianImpedanceResultConstPtr & result)
{
    ROS_INFO("Reach action for task '%s' has been completed",
             getName().c_str());
}
