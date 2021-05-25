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

void InteractionRos::getForceLimits (Eigen::Vector6d& fmin, Eigen::Vector6d& fmax) const
{
	ROS_WARN("unsupported function: getForceLimits");
	
	fmin.setZero();
	fmax.setZero();
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

bool InteractionRos::setForceLimits (const Eigen::Vector6d& fmin, const Eigen::Vector6d& fmax)
{
	ROS_WARN("unsupported function: getForceLimits");
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
