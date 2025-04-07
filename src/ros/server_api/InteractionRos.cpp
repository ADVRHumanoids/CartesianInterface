#include "fmt/format.h"

#include "ros/server_api/InteractionRos.h"

#include <tf2_eigen/tf2_eigen.hpp>

#include <xbot2_interface/logger.h>

using namespace XBot::Cartesian;
using namespace XBot::Cartesian::ServerApi;
using XBot::Logger;

RCIAManager::RCIAManager(rclcpp::Node::SharedPtr node,
						 InteractionTask::Ptr task):
	
    // _server (new ActionServer(nh, task->getName() + "/stiffness", false)),
    _task(task),
    _state(ReachActionState::IDLE),
    _name(task->getName()),
    _node(node)
{
    using namespace std::placeholders;
    _server = rclcpp_action::create_server<ReachCartesianImpedance>(
                node,
                task->getName() + "/stiffness",
                std::bind(&RCIAManager::handle_goal, this, _1, _2),
                std::bind(&RCIAManager::handle_cancel, this, _1),
                std::bind(&RCIAManager::handle_accepted, this, _1)
                );
}

rclcpp_action::GoalResponse RCIAManager::handle_goal(const rclcpp_action::GoalUUID &uuid,
                                                     std::shared_ptr<const ReachCartesianImpedance::Goal> goal)
{
    Logger::info(Logger::Severity::HIGH,
                     "Received new goal for task '%s'\n", _name.c_str());

    if (goal->target.size() == 0)
    {
        Logger::error("Invalid goal received for task '%s' •`_´• \n", _name.c_str());

        return rclcpp_action::GoalResponse::REJECT;
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

        tf2::fromMsg(goal->target[k].impedance.linear.stiffness, temp1);
        tf2::fromMsg(goal->target[k].impedance.angular.stiffness, temp2);
        
        stiffness.head(3) = temp1; stiffness.tail(3) = temp2;
        
        Interpolator<Eigen::Matrix6d>::WayPoint wp;
        
        wp.value = stiffness.asDiagonal();
        wp.time  = goal->target[k].time;
        
        waypoints.push_back(wp);
    }

    // send waypoints to cartesian ifc
    if(!_task->setStiffnessTransition(waypoints))
    {
        Logger::error("Something went wrong (>_<)'");
        return rclcpp_action::GoalResponse::REJECT; // next state is 'idle'
    }
    
    _fb = std::make_shared<ReachCartesianImpedance::Feedback>();

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse RCIAManager::handle_cancel(const std::shared_ptr<GoalHandle> goal_handle)
{
    Logger::info(Logger::Severity::HIGH,
                     "Received request to cancel goal");

    _goal_handle.reset();

    return rclcpp_action::CancelResponse::ACCEPT;
}

void RCIAManager::handle_accepted(const std::shared_ptr<GoalHandle> goal_handle)
{
    _goal_handle = goal_handle;
    _state = ReachActionState::ACCEPTED;
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
    // Note: Implementation moved to handle_goal
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
    if (_goal_handle->is_canceling())
    {
		XBot::Logger::info(XBot::Logger::Severity::HIGH,
                           "Goal for task '%s' canceled by user\n",
                           _name.c_str());

        _task->abortStiffnessTransition();

        auto result = std::make_shared<ReachCartesianImpedance::Result>();

		/* use this affine in case you want to transform the stiffness in another frame */
        /* Eigen::Affine3d base_T_ee;
        _task->getCurrentPose(base_T_ee); */
		
		Impedance impedance = _task->getImpedance();
		
		tf2::toMsg (impedance.stiffness.diagonal().head(3), result->impedance_final.linear.stiffness);
        tf2::toMsg (impedance.stiffness.diagonal().tail(3), result->impedance_final.angular.stiffness);
		
		tf2::toMsg (impedance.damping.diagonal().head(3), result->impedance_final.linear.damping_ratio);
        tf2::toMsg (impedance.damping.diagonal().tail(3), result->impedance_final.angular.damping_ratio);
		
		// i don't fill the other fields of the result

        _goal_handle->canceled(result);
        _state = ReachActionState::COMPLETED; // next state is 'completed'
        return;
    }

    // trajectory ended
    if(_task->getStiffnessState() == State::Online)
    {
		auto result = std::make_shared<ReachCartesianImpedance::Result>();
		
        /* use this affine in case you want to transform the stiffness in another frame */
        Eigen::Affine3d base_T_ee;
        _task->getCurrentPose(base_T_ee);
		
		Impedance impedance = _task->getImpedance();
		
        tf2::toMsg (impedance.stiffness.diagonal().head(3), result->impedance_final.linear.stiffness);
        tf2::toMsg (impedance.stiffness.diagonal().tail(3), result->impedance_final.angular.stiffness);
		
		tf2::toMsg (impedance.damping.diagonal().head(3), result->impedance_final.linear.damping_ratio);
        tf2::toMsg (impedance.damping.diagonal().tail(3), result->impedance_final.angular.damping_ratio);
				
		XBot::Logger::success(XBot::Logger::Severity::HIGH,
                              "Impedance for task '%s' updated!",
                              _name.c_str());

        _goal_handle->succeed(result);
        _goal_handle.reset();
        _state = ReachActionState::COMPLETED; // next state is 'completed'
        return;
    }
    else // publish feedback
    {
		Impedance impedance = _task->getImpedance();
		
        tf2::toMsg (impedance.stiffness.diagonal().head(3), _fb->impedance_actual.linear.stiffness);
        tf2::toMsg (impedance.stiffness.diagonal().tail(3), _fb->impedance_actual.angular.stiffness);
		
		tf2::toMsg (impedance.damping.diagonal().head(3), _fb->impedance_actual.linear.damping_ratio);
        tf2::toMsg (impedance.damping.diagonal().tail(3), _fb->impedance_actual.angular.damping_ratio);

		
        /*feedback.impedance_actual.header.stamp = ros::Time::now();
        feedback.current_reference.header.frame_id = _task->getBaseLink();*/
        
		_fb->time_to_finish = -1.0;
		_fb->progress = -1.0;

        _goal_handle->publish_feedback(_fb);
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
	using namespace std::placeholders;

	registerType("Interaction");
	
	_ci_inter = task;

    if(!_ci_inter)
    {
        throw std::runtime_error("Provided task does not have expected type 'InteractionTask'");
    }

	auto n = _ctx->node();

    _action = std::make_unique<RCIAManager>(n, _ci_inter);

	_impd_pub = n->create_publisher<CartesianImpedance>(task->getName() + "/current_impedance"      , 1);
    _fref_pub = n->create_publisher<WrenchStamped>(task->getName() + "/current_force_reference", 1);

    _task_info_pub = n->create_publisher<InteractionTaskInfo>(
                _task->getName() + "/interaction_task_properties", 1
                );

    _fref_sub = n->create_subscription<WrenchStamped>(task->getName() + "/force_reference", 1,
                                                                      std::bind(&InteractionRos::on_fref_recv, this, _1));
	
	_get_info_srv = n->create_service<GetInteractionTaskInfo>(_task->getName() + "/get_interaction_task_properties",
                                                             std::bind( &InteractionRos::get_task_info_cb, this, _1, _2 ));
	
	_get_impedance_srv = n->create_service<GetImpedance>(_task->getName() + "/get_impedance",
                                                     std::bind(&InteractionRos::get_impedance_cb, this, _1, _2));

    _set_impedance_srv = n->create_service<SetImpedance>(_task->getName() + "/set_impedance",
                                                     std::bind(&InteractionRos::set_impedance_cb, this, _1, _2));

    _set_impedance_ref_link_srv = n->create_service<SetImpedanceRefLink>(_task->getName() + "/set_impedance_ref_link",
                                                     std::bind( &InteractionRos::set_impedance_ref_link_cb, this, _1, _2));

    _set_force_limits_srv = n->create_service<SetForceLimits>(task->getName() + "/set_force_limits",
                                                              std::bind(&InteractionRos::set_force_limits_cb, this, _1, _2));
    
    _get_force_limits_srv = n->create_service<GetForceLimits>(_task->getName() + "/get_force_limits",
                                                     std::bind(&InteractionRos::get_force_limits_cb, this, _1, _2));
}

bool InteractionRos::get_task_info_cb(GetInteractionTaskInfo::Request::ConstSharedPtr  req,
									  GetInteractionTaskInfo::Response::SharedPtr res)
{
	res->state = EnumToString(_ci_inter->getStiffnessState());
    res->impedance_ref_link = _ci_inter->getImpedanceRefLink();
    
    return true;
}

bool InteractionRos::get_impedance_cb(GetImpedance::Request::ConstSharedPtr req,
									  GetImpedance::Response::SharedPtr res)
{
	Impedance impedance = _ci_inter->getImpedance();

    tf2::toMsg (impedance.stiffness.diagonal().head(3), res->impedance.linear.stiffness);
	tf2::toMsg (impedance.stiffness.diagonal().tail(3), res->impedance.angular.stiffness);

	tf2::toMsg (impedance.damping.diagonal().head(3), res->impedance.linear.damping_ratio);
	tf2::toMsg (impedance.damping.diagonal().tail(3), res->impedance.angular.damping_ratio);

    res->impedance.header.frame_id = _ci_inter->getImpedanceRefLink();
    
    return true;
}

bool InteractionRos::set_impedance_cb(SetImpedance::Request::ConstSharedPtr req,
                                      SetImpedance::Response::SharedPtr res)

{
    Eigen::Vector3d lin_stiff, ang_stiff, lin_damp, ang_damp;

    tf2::fromMsg(req->impedance.linear.stiffness, lin_stiff);
    tf2::fromMsg(req->impedance.angular.stiffness, ang_stiff);

    tf2::fromMsg(req->impedance.linear.damping_ratio, lin_damp);
    tf2::fromMsg(req->impedance.angular.damping_ratio, ang_damp);

    Eigen::Matrix6d stiffness, damping;

    damping.  setZero();
    stiffness.setZero();

    stiffness.diagonal().head(3) = lin_stiff;
    stiffness.diagonal().tail(3) = ang_stiff;

    damping.diagonal().head(3) = lin_damp;
    damping.diagonal().tail(3) = ang_damp;

    //* TODO set mass matrix
    Logger::warning(Logger::Severity::HIGH, "Unsupported feat: set mass matrix");

    Impedance impedance(stiffness, damping);

    if (_ci_inter->setImpedance(impedance))
    {
        res->message = fmt::format("Successfully set impedance");   // to:\n{}", impedance);
        res->success = true;
        return true;
    }

    else
    {
        res->message = fmt::format("Unable to set impedance");   // to:\n{}", impedance);
        res->success = false;
        return false;
    }

}

bool InteractionRos::get_force_limits_cb(GetForceLimits::Request::ConstSharedPtr  req,
									     GetForceLimits::Response::SharedPtr res)
{
    Eigen::Vector6d fmax;
    _ci_inter->getForceLimits(fmax);

    tf2::toMsg(fmax.head(3), res->fmax.force);
    tf2::toMsg(fmax.tail(3), res->fmax.torque);

    return true;
}

bool InteractionRos::set_force_limits_cb(SetForceLimits::Request::ConstSharedPtr req,
                                         SetForceLimits::Response::SharedPtr res)

{
    Eigen::Vector3d force, torque;
    Eigen::Vector6d fmax;

    tf2::fromMsg(req->fmax.force, force);
    tf2::fromMsg(req->fmax.torque, torque);

    fmax << force, torque;

    if (_ci_inter->setForceLimits(fmax))
    {
        res->message = fmt::format("Successfully set force limits"); // to: {}", fmax);
        res->success = true;
        return true;
    }

    else
    {
        res->message = fmt::format("Unable to set force limits"); // to: {}", fmax);
        res->success = false;
        return false;
    }

}

void InteractionRos::run(rclcpp::Time time)
{
    CartesianRos::run(time);
	
	_action->run();
	
    geometry_msgs::msg::WrenchStamped fr;
    tf2::toMsg(_ci_inter->getForceReference().head(3), fr.wrench.force);
	tf2::toMsg(_ci_inter->getForceReference().tail(3), fr.wrench.torque);
    
	CartesianImpedance cimp;
	
	Impedance impedance = _ci_inter->getImpedance();
		
	tf2::toMsg (impedance.stiffness.diagonal().head(3), cimp.linear.stiffness);
	tf2::toMsg (impedance.stiffness.diagonal().tail(3), cimp.angular.stiffness);
	
	tf2::toMsg (impedance.damping.diagonal().head(3), cimp.linear.damping_ratio);
	tf2::toMsg (impedance.damping.diagonal().tail(3), cimp.angular.damping_ratio);
	
    _fref_pub->publish(fr);
	_impd_pub->publish(cimp);

    publish_task_info();
}

void InteractionRos::on_fref_recv(WrenchStamped::ConstSharedPtr msg)
{
    Eigen::Vector6d fref = Eigen::Vector6d::Zero();
    Eigen::Vector3d force = Eigen::Vector3d::Zero();
    Eigen::Vector3d torque = Eigen::Vector3d::Zero();

    tf2::fromMsg(msg->wrench.force, force);
    tf2::fromMsg(msg->wrench.torque, torque);    
    fref << force, torque;

    _ci_inter->setForceReference(fref);
}

void InteractionRos::publish_task_info()
{
    auto info_req = std::make_shared<GetInteractionTaskInfo::Request>();
    auto info_res = std::make_shared<GetInteractionTaskInfo::Response>();

    get_task_info_cb(info_req, info_res);

    InteractionTaskInfo msg;
    msg.state = info_res->state;
    msg.impedance_ref_link = info_res->impedance_ref_link;

    _task_info_pub->publish(msg);

}

bool InteractionRos::set_impedance_ref_link_cb(SetImpedanceRefLink::Request::ConstSharedPtr req,
                                               SetImpedanceRefLink::Response::SharedPtr res)
{
    auto old_impedance_ref_link = _ci_inter->getImpedanceRefLink();
    res->success = _ci_inter->setImpedanceRefLink(req->impedance_ref_link);

    if(res->success)
    {
        res->message = fmt::format("Successfully changed impedance ref link from '{}' to '{}' for task '{}'",
                                  old_impedance_ref_link, _ci_inter->getImpedanceRefLink(), _ci_inter->getName());
    }
    else
    {
        res->message = fmt::format("Unable to change impedance ref link from '{}' to '{}' for task '{}'",
                                  old_impedance_ref_link, req->impedance_ref_link, _ci_inter->getName());
    }

    return true;
}