#include "fmt/format.h"
#include "ros/client_api/InteractionRos.h"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <xbot2_interface/logger.h>

using XBot::Logger;
using namespace XBot::Cartesian;
using namespace XBot::Cartesian::ClientApi;
using namespace cartesian_interface_ros;
using namespace std::chrono_literals;

InteractionRos::InteractionRos(std::string name,
							   rclcpp::Node::SharedPtr node):
    CartesianRos(name, node)
{
    _action_cli = rclcpp_action::create_client<ReachCartesianImpedance>(_node, name + "/stiffness");
    _interaction_info_cli = _node->create_client<GetInteractionTaskInfo>(name + "/get_interaction_task_properties");

    // Ugly: Too many blocking calls?
    while(!_action_cli->wait_for_action_server(1s))
    {
        RCLCPP_INFO_STREAM(_node->get_logger(),
                           "Waiting for action server" << name << "/stiffness"
                           );
    }

    while(!_interaction_info_cli->wait_for_service(1s))
    {
        RCLCPP_INFO_STREAM(_node->get_logger(),
                           fmt::format("Waiting for service '{}'",
                                       _interaction_info_cli->get_service_name())
                           );
    }
    
    _get_impedance_cli = _node->create_client<GetImpedance>(name + "/get_impedance");

    // : wait for service to be available

    while(!_get_impedance_cli->wait_for_service(1s))
    {
        RCLCPP_INFO_STREAM(_node->get_logger(),
                           fmt::format("Waiting for service '{}'",
                                       _get_impedance_cli->get_service_name())
                           );
    }

    _set_impedance_cli = _node->create_client<SetImpedance>(name + "/set_impedance");

    while(!_set_impedance_cli->wait_for_service(1s))
    {
        RCLCPP_INFO_STREAM(_node->get_logger(),
                           fmt::format("Waiting for service '{}'",
                                       _set_impedance_cli->get_service_name())
                           );
    }

    _set_impedance_ref_link_cli = _node->create_client<SetImpedanceRefLink>(name + "/set_impedance_ref_link");
    
    while(!_set_impedance_ref_link_cli->wait_for_service(1s))
    {
        RCLCPP_INFO_STREAM(_node->get_logger(),
                           fmt::format("Waiting for service '{}'",
                                       _set_impedance_ref_link_cli->get_service_name())
                           );
    }

    _get_force_limits_cli = _node->create_client<GetForceLimits>(name + "/get_force_limits");
    
    while(!_get_force_limits_cli->wait_for_service(1s))
    {
        RCLCPP_INFO_STREAM(_node->get_logger(),
                           fmt::format("Waiting for service '{}'",
                                       _get_force_limits_cli->get_service_name())
                           );
    }

    _set_force_limits_cli = _node->create_client<SetForceLimits>(name + "/set_force_limits");
    
    while(!_set_force_limits_cli->wait_for_service(1s))
    {
        RCLCPP_INFO_STREAM(_node->get_logger(),
                           fmt::format("Waiting for service '{}'",
                                       _set_force_limits_cli->get_service_name())
                            );
    }

    _task_info_sub = _node->create_subscription<InteractionTaskInfo>(name + "/interaction_task_properties",
                                                                    10,
                                                                    std::bind(&InteractionRos::on_task_info_recv, this, std::placeholders::_1)); 
    
    _f.setZero();
}

GetInteractionTaskInfo::Response::SharedPtr InteractionRos::get_task_info() const
{
    if(asyncMode())
    {
        GetInteractionTaskInfo::Response::SharedPtr res;
        
		res->state = _info.state;
        
        return res;
    }

    auto req = std::make_shared<GetInteractionTaskInfo::Request>();
    auto cli = _interaction_info_cli;
    auto fut = cli->async_send_request(req);

    if(rclcpp::spin_until_future_complete(_node, fut, 1s) == rclcpp::FutureReturnCode::SUCCESS)
    {
        return fut.get();
    }
    else
    {
        throw std::runtime_error(fmt::format("Unable to call service '{}'",
                                             cli->get_service_name()));
    }

}

const Impedance & InteractionRos::getImpedance()
{
	// cartesian_interface::GetImpedance srv;
    // if(!_get_impedance_cli.call(srv))
    // {
        // throw std::runtime_error(fmt::format("Unable to call service '{}'",
                                            //  _get_impedance_cli.getService()));
    // }

    auto req = std::make_shared<GetImpedance::Request>();
    auto fut = _get_impedance_cli->async_send_request(req);

    if(rclcpp::spin_until_future_complete(_node, fut, 1s) == rclcpp::FutureReturnCode::SUCCESS)
    {
        // get current state for task (note: should it be getPoseReference instead?)
        Eigen::Vector3d temp1, temp2;
        Eigen::Vector6d temp3;

        tf2::fromMsg(fut.get()->impedance.linear.stiffness,  temp1);
        tf2::fromMsg(fut.get()->impedance.angular.stiffness, temp2);
                
        temp3.head(3) = temp1; temp3.tail(3) = temp2;
        
        _impedance.stiffness = temp3.asDiagonal();
        
        tf2::fromMsg(fut.get()->impedance.linear.damping_ratio,  temp1);
        tf2::fromMsg(fut.get()->impedance.angular.damping_ratio, temp2);
        
        temp3.head(3) = temp1; temp3.tail(3) = temp2;
        
        _impedance.damping = temp3.asDiagonal();
        
        _impedance.mass.setZero();
        
        return _impedance;
    }
    else
    {
        throw std::runtime_error(fmt::format("Unable to call service '{}'",
                                             _get_impedance_cli->get_service_name()));
    }
}

const Eigen::Vector6d& InteractionRos::getForceReference () const
{
	Logger::warning(Logger::Severity::MID, "Unsupported function: getForceReference()");
	return _f;
}

bool InteractionRos::setImpedance (const Impedance & impedance)
{
    // cartesian_interface::SetImpedance srv;

    auto req = std::make_shared<SetImpedance::Request>();

    tf2::toMsg(impedance.stiffness.diagonal().head(3), req->impedance.linear.stiffness);
    tf2::toMsg(impedance.stiffness.diagonal().tail(3), req->impedance.angular.stiffness);

    tf2::toMsg(impedance.damping.diagonal().head(3), req->impedance.linear.damping_ratio);
    tf2::toMsg(impedance.damping.diagonal().tail(3), req->impedance.angular.damping_ratio);
    
    auto cli = _set_impedance_cli;
    auto fut = cli->async_send_request(req);

    if((rclcpp::spin_until_future_complete(_node, fut, 1s) == rclcpp::FutureReturnCode::SUCCESS))
    {
        RCLCPP_INFO_STREAM(_node->get_logger(), fut.get()->message);

        return fut.get()->success;
    }
    else
    {
        throw std::runtime_error(fmt::format("Unable to call service '{}'",
                                             cli->get_service_name()));
    }
}

void InteractionRos::setForceReference (const Eigen::Vector6d& f)
{
	Logger::warning(Logger::Severity::MID, "Unsupported function: setForceReference");
}

void InteractionRos::getForceLimits (Eigen::Vector6d& fmax) const
{
	// cartesian_interface::GetForceLimits srv;
    auto req = std::make_shared<GetForceLimits::Request>();
    auto cli = _get_force_limits_cli;
    auto fut = cli->async_send_request(req);

    if(rclcpp::spin_until_future_complete(_node, fut, 1s) == rclcpp::FutureReturnCode::SUCCESS)
    {
        // Note: get current state for task (should it be getPoseReference instead?)
        Eigen::Vector3d force, torque;
            
        tf2::fromMsg(fut.get()->fmax.force, force);
        tf2::fromMsg(fut.get()->fmax.torque, torque);
                
        fmax << force, torque;
    }
    else 
    {
        throw std::runtime_error(fmt::format("Unable to call service '{}'",
                                             cli->get_service_name()));
    }
}

bool InteractionRos::setForceLimits (const Eigen::Vector6d& fmax)
{
    
    auto req = std::make_shared<SetForceLimits::Request>();
    
    tf2::toMsg(fmax.head(3), req->fmax.force);
    tf2::toMsg(fmax.tail(3), req->fmax.torque);
    
    auto cli = _set_force_limits_cli;
    auto fut = cli->async_send_request(req);
    
    if (rclcpp::spin_until_future_complete(_node, fut, 1s) == rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_INFO_STREAM(_node->get_logger(), fut.get()->message);
        
        return fut.get()->success;
    }
    else
    {
        throw std::runtime_error(fmt::format("Unable to call service '{}'",
                                             cli->get_service_name()));
    }
}

State InteractionRos::getStiffnessState() const
{
    return StringToEnum<State>(get_task_info()->state);
}

void InteractionRos::abortStiffnessTransition()
{
    auto fut = _action_cli->async_cancel_all_goals();
    if (rclcpp::spin_until_future_complete(_node, fut, 1s) == rclcpp::FutureReturnCode::SUCCESS)
    {
        Logger::info(Logger::Severity::HIGH, "Abort succeeded");
    }
    else    
    {
        Logger::error(Logger::Severity::HIGH, "Abort failed");
    }
    
}

bool InteractionRos::waitTransitionCompleted(double timeout)
{
    auto ret = rclcpp::spin_until_future_complete(_node,
                                                  _action_future,
                                                  std::chrono::duration<double>(timeout));
    return ret == rclcpp::FutureReturnCode::SUCCESS;
}

bool InteractionRos::setStiffnessTransition(const Interpolator<Eigen::Matrix6d>::WayPointVector & way_points)
{
    // cartesian_interface::ReachCartesianImpedanceGoal goal;

    if (!_action_cli->action_server_is_ready())
    {
        Logger::error(Logger::Severity::HIGH, "Server not ready");
        return false;
    }

    ReachCartesianImpedance::Goal goal;
	Impedance impedance = getImpedance();
	
	for(const auto& wp : way_points)
    {
		CartesianImpedanceTimed cit;
		
		tf2::toMsg (wp.value.diagonal().head(3), cit.impedance.linear.stiffness );
		tf2::toMsg (wp.value.diagonal().tail(3), cit.impedance.angular.stiffness);
		
		tf2::toMsg (impedance.damping.diagonal().head(3), cit.impedance.linear.damping_ratio );
		tf2::toMsg (impedance.damping.diagonal().tail(3), cit.impedance.angular.damping_ratio);
		
		cit.time = wp.time;
		
		goal.target.push_back(cit);
    }

    auto send_goal_option = rclcpp_action::Client<ReachCartesianImpedance>::SendGoalOptions();

    using namespace std::placeholders;

    send_goal_option.goal_response_callback = 
        std::bind(&InteractionRos::on_action_active, this, _1);

    send_goal_option.feedback_callback = 
        std::bind(&InteractionRos::on_action_feedback, this, _1, _2);

    send_goal_option.result_callback = 
        std::bind(&InteractionRos::on_action_done, this, _1);
    
    return true;
}

const std::string & InteractionRos::getImpedanceRefLink() const
{
    _impedance_ref_link = get_task_info()->impedance_ref_link;
    return _impedance_ref_link;
}

bool InteractionRos::setImpedanceRefLink(const std::string & new_impedance_ref_link)
{

    auto req = std::make_shared<SetImpedanceRefLink::Request>();
    req->impedance_ref_link = new_impedance_ref_link;
    
    auto cli = _set_impedance_ref_link_cli;
    auto fut = cli->async_send_request(req);

    if (rclcpp::spin_until_future_complete(_node, fut, 1s) == rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_INFO_STREAM(_node->get_logger(), fut.get()->message);
        
        return fut.get()->success;
    }
    else
    {
        throw std::runtime_error(fmt::format("Unable to call service '{}'",
                                             cli->get_service_name()));
    }

}

void InteractionRos::on_task_info_recv(InteractionTaskInfo::ConstSharedPtr msg)
{
    _info = *msg;
}

void InteractionRos::on_action_feedback(ActionGoalHandle::SharedPtr goal_handle,
                                        const std::shared_ptr<const ReachCartesianImpedance::Feedback> feedback)
{
    // TODO: Implement Feedback?
}

void InteractionRos::on_action_active(ActionGoalHandle::SharedPtr goal_handle)
{
    RCLCPP_INFO(_node->get_logger(),
                "reach action for task '%s' has become active",
                getName().c_str());

    _action_goal_handle = goal_handle;
}

void InteractionRos::on_action_done(const ActionGoalHandle::WrappedResult& result)
{
    RCLCPP_INFO(_node->get_logger(),
                "reach action for task '%s' has been completed",
                getName().c_str());
                
    _action_goal_handle.reset();
}
