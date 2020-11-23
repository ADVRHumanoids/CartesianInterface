#include "ros/server_api/InteractionRos.h"
#include <eigen_conversions/eigen_msg.h>


using namespace XBot::Cartesian;
using namespace XBot::Cartesian::ServerApi;


RCIAManager::RCIAManager(ros::NodeHandle      nh,
						 InteractionTask::Ptr task):
	
    _server (new ActionServer(nh, task->getName() + "/stiffness", false)),
    _task   (task),
    _state  (ReachActionState::IDLE),
    _name   (task->getName())
{
    _server->start();
}

void RCIAManager::run()
{
    switch(_state)
    {
		
    case ReachActionState::IDLE:
        run_state_idle();
        break;
		
    case ReachActionState::ACCEPTED:
        run_state_accepted();
        break;
		
    case ReachActionState::RUNNING:
        run_state_running();
        break;
		
    case ReachActionState::COMPLETED:
        run_state_completed();
        break;
    }
}

void RCIAManager::run_state_idle()
{
    // wait for new goal to be available,
    // then transit to 'accepted'

    if(_server->isNewGoalAvailable())
    {
		Logger::info(Logger::Severity::HIGH,
                     "Received new goal for task '%s'\n", _name.c_str());

        // obtain new goal
        auto goal = _server->acceptNewGoal();

        // check consistency
        if (goal->target.size() == 0)
        {
            _server->setAborted(cartesian_interface::ReachCartesianImpedanceResult(),
                                "Empty traget!");

            Logger::error("Invalid goal received for task '%s' \n", _name.c_str());

            return; // next state is 'idle'
        }

        Logger::info(Logger::Severity::HIGH,
                     "Accepted new goal for task '%s'\n", _name.c_str());

        // get current state for task (note: should it be getPoseReference instead?)
        Impedance impedance = _task->getImpedance();
		
		// fill waypoint vector
        Interpolator<Eigen::Matrix6d>::WayPointVector waypoints;

        for(int k = 0; k < goal->target.size(); k++)
        {
            Eigen::Vector3d temp1, temp2;
			Eigen::Vector6d stiffness;
			
			tf::vectorMsgToEigen(goal->target[k].impedance.linear.stiffness, temp1);
            tf::vectorMsgToEigen(goal->target[k].impedance.angular.stiffness, temp2);
			
			stiffness.head(3) = temp1; stiffness.tail(3) = temp2;
			
            Interpolator<Eigen::Matrix6d>::WayPoint wp;
			
            wp.value = stiffness.asDiagonal();
            wp.time  = goal->target[k].time;
			
            waypoints.push_back(wp);
        }

        // send waypoints to cartesian ifc
        if(!_task->setStiffnessTransition(waypoints))
        {
			_server->setAborted(cartesian_interface::ReachCartesianImpedanceResult(), "Internal error");
            return; // next state is 'idle'
        }
        
        // transit to 'accepted'
        _state = ReachActionState::ACCEPTED;
        return;
    }
}

void RCIAManager::run_state_accepted()
{
    // wait till cartesian ifc switches state to 'reaching'
    if(_task->getStiffnessState() == State::Reaching)
    {
		Logger::info(Logger::Severity::HIGH,
                     "Reaching started for task '%s'\n", _name.c_str());

        _state = ReachActionState::RUNNING; // next state is 'running'
        return;
    }

}

void RCIAManager::run_state_running()
{
	// manage preemption
    if (_server->isPreemptRequested())
    {
		XBot::Logger::info(XBot::Logger::Severity::HIGH,
                           "Goal for task '%s' canceled by user\n",
                           _name.c_str());

        _task->abortStiffnessTransition();

        cartesian_interface::ReachCartesianImpedanceResult result;
		
		/* use this affine in case you want to transform the stiffness in another frame */
        Eigen::Affine3d base_T_ee;
        _task->getCurrentPose(base_T_ee);
		
		Impedance impedance = _task->getImpedance();
		
		tf::vectorEigenToMsg (impedance.stiffness.diagonal().head(3), result.impedance_final.linear.stiffness);
        tf::vectorEigenToMsg (impedance.stiffness.diagonal().tail(3), result.impedance_final.angular.stiffness);
		
		tf::vectorEigenToMsg (impedance.damping.diagonal().head(3), result.impedance_final.linear.damping_ratio);
        tf::vectorEigenToMsg (impedance.damping.diagonal().tail(3), result.impedance_final.angular.damping_ratio);
		
		// i don't fill the other fields of the result

        _server->setPreempted(result);
        _state = ReachActionState::COMPLETED; // next state is 'completed'
        return;
    }

    // trajectory ended
    if(_task->getStiffnessState() == State::Online)
    {
		cartesian_interface::ReachCartesianImpedanceResult result;
		
        /* use this affine in case you want to transform the stiffness in another frame */
        Eigen::Affine3d base_T_ee;
        _task->getCurrentPose(base_T_ee);
		
		Impedance impedance = _task->getImpedance();
		
        tf::vectorEigenToMsg (impedance.stiffness.diagonal().head(3), result.impedance_final.linear.stiffness);
        tf::vectorEigenToMsg (impedance.stiffness.diagonal().tail(3), result.impedance_final.angular.stiffness);
		
		tf::vectorEigenToMsg (impedance.damping.diagonal().head(3), result.impedance_final.linear.damping_ratio);
        tf::vectorEigenToMsg (impedance.damping.diagonal().tail(3), result.impedance_final.angular.damping_ratio);
				
		XBot::Logger::success(XBot::Logger::Severity::HIGH,
                              "Impedance for task '%s' updated!",
                              _name.c_str());

        _server->setSucceeded(result);
        _state = ReachActionState::COMPLETED; // next state is 'completed'
        return;
    }
    else // publish feedback
    {
		cartesian_interface::ReachCartesianImpedanceFeedback feedback;

		Impedance impedance = _task->getImpedance();
		
        tf::vectorEigenToMsg (impedance.stiffness.diagonal().head(3), feedback.impedance_actual.linear.stiffness);
        tf::vectorEigenToMsg (impedance.stiffness.diagonal().tail(3), feedback.impedance_actual.angular.stiffness);
		
		tf::vectorEigenToMsg (impedance.damping.diagonal().head(3), feedback.impedance_actual.linear.damping_ratio);
        tf::vectorEigenToMsg (impedance.damping.diagonal().tail(3), feedback.impedance_actual.angular.damping_ratio);

		
        /*feedback.impedance_actual.header.stamp = ros::Time::now();
        feedback.current_reference.header.frame_id = _task->getBaseLink();*/
        
		feedback.time_to_finish = -1.0;
		feedback.progress = -1.0;

        _server->publishFeedback(feedback);
        return; // next state is 'running'

    }
}

void RCIAManager::run_state_completed()
{
	XBot::Logger::info(XBot::Logger::Severity::HIGH,
                       "Goal for task '%s' completed\n",
                       _name.c_str());

    _state = ReachActionState::IDLE;
    return;
}

InteractionRos::InteractionRos(InteractionTask::Ptr task,
                               RosContext::Ptr context):
    CartesianRos(task, context),
    _ci_inter(task)
{
	
	registerType("Interaction");
	
	_ci_inter = task;

    if(!_ci_inter)
    {
        throw std::runtime_error("Provided task does not have expected type 'InteractionTask'");
    }

	_action.reset(new RCIAManager(_ctx->nh(), _ci_inter));
	
	_impd_pub = _ctx->nh().advertise<cartesian_interface::CartesianImpedance>(task->getName() + "/current_impedance"      , 1);
    _fref_pub = _ctx->nh().advertise<geometry_msgs::WrenchStamped           >(task->getName() + "/current_force_reference", 1);
	
    _fref_sub = _ctx->nh().subscribe(task->getName() + "/force_reference", 1, &InteractionRos::on_fref_recv, this);
	
	_get_info_srv = _ctx->nh().advertiseService(_task->getName() + "/get_interaction_task_properties",
                                                &InteractionRos::get_task_info_cb, this);
	
	_get_impedance_srv = _ctx->nh().advertiseService(_task->getName() + "/get_impedance",
                                                &InteractionRos::get_impedance_cb, this);
}

bool InteractionRos::get_task_info_cb(cartesian_interface::GetInteractionTaskInfoRequest&  req,
									  cartesian_interface::GetInteractionTaskInfoResponse& res)
{
	res.state = EnumToString(_ci_inter->getTaskState());
}

bool InteractionRos::get_impedance_cb(cartesian_interface::GetImpedanceRequest&  req,
									  cartesian_interface::GetImpedanceResponse& res)
{
	Impedance impedance = _ci_inter->getImpedance();
	
	tf::vectorEigenToMsg (impedance.stiffness.diagonal().head(3), res.impedance.linear.stiffness);
	tf::vectorEigenToMsg (impedance.stiffness.diagonal().tail(3), res.impedance.angular.stiffness);
	
	tf::vectorEigenToMsg (impedance.damping.diagonal().head(3), res.impedance.linear.damping_ratio);
	tf::vectorEigenToMsg (impedance.damping.diagonal().tail(3), res.impedance.angular.damping_ratio);
}

void InteractionRos::run(ros::Time time)
{
    CartesianRos::run(time);
	
	_action->run();
	
    geometry_msgs::WrenchStamped fr;
    tf::wrenchEigenToMsg(_ci_inter->getForceReference(), fr.wrench);
	
	cartesian_interface::CartesianImpedance cimp;
	
	Impedance impedance = _ci_inter->getImpedance();
		
	tf::vectorEigenToMsg (impedance.stiffness.diagonal().head(3), cimp.linear.stiffness);
	tf::vectorEigenToMsg (impedance.stiffness.diagonal().tail(3), cimp.angular.stiffness);
	
	tf::vectorEigenToMsg (impedance.damping.diagonal().head(3), cimp.linear.damping_ratio);
	tf::vectorEigenToMsg (impedance.damping.diagonal().tail(3), cimp.angular.damping_ratio);
	
    _fref_pub.publish(fr);
	_impd_pub.publish(cimp);
}

void InteractionRos::on_fref_recv(geometry_msgs::WrenchStampedConstPtr msg)
{
    Eigen::Vector6d fref;
    tf::wrenchMsgToEigen(msg->wrench, fref);

    _ci_inter->setForceReference(fref);
}
