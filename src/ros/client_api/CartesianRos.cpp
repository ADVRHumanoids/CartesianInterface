#include "fmt/format.h"
#include "ros/client_api/CartesianRos.h"
#include "../utils/RosUtils.h"
#include <tf2_eigen/tf2_eigen.hpp>

using namespace XBot::Cartesian;
using namespace XBot::Cartesian::ClientApi;
using namespace cartesian_interface;
using namespace std::chrono_literals;

CartesianRos::CartesianRos(std::string name,
                           rclcpp::Node::SharedPtr node):
    TaskRos(name, node),
    _Tref_recv(false), _vref_recv(false)
{
    _action_cli = rclcpp_action::create_client<ReachPose>(_node, name + "/reach");

    _cart_info_cli = _node->create_client<GetCartesianTaskInfo>(name + "/get_cartesian_task_properties");

    while(!_cart_info_cli->wait_for_service(1s))
    {
        RCLCPP_INFO_STREAM(_node->get_logger(),
                           fmt::format("waiting for service '{}'",
                                       _cart_info_cli->get_service_name())
                           );
    }

    while(!_action_cli->wait_for_action_server(1s))
    {
        RCLCPP_INFO_STREAM(_node->get_logger(),
                           "waiting for action server" << name << "/reach"
                           );
    }

    _set_base_link_cli = _node->create_client<SetBaseLink>(name + "/set_base_link");

    _set_ctrl_mode_cli = _node->create_client<SetControlMode>(name + "/set_control_mode");

    _pose_ref_pub = _node->create_publisher<PoseStamped>(name + "/reference", 1);

    _vel_ref_pub = _node->create_publisher<TwistStamped>(name + "/velocity_reference", 1);

    auto on_ref_recv = [this](PoseStamped::ConstSharedPtr msg)
    {
        _Tref_recv = true;
        tf2::fromMsg(msg->pose, _Tref);
    };

    _pose_ref_sub = _node->create_subscription<PoseStamped>(name + "/current_reference",
                                                            1, on_ref_recv
                                                            );

    auto on_vref_recv = [this](TwistStamped::ConstSharedPtr msg)
    {
        _vref_recv = true;
        tf2::fromMsg(msg->twist, _vref);
    };

    _vel_ref_sub = _node->create_subscription<TwistStamped>(name + "/current_velocity_reference", 1,
                                                            on_vref_recv);

    _set_safety_lims_cli = _node->create_client<SetSafetyLimits>(name + "/set_safety_limits");

    _task_info_sub = ::create_subscription<CartesianTaskInfo>(_node,
                                                              name + "/cartesian_task_properties",
                                                              &CartesianRos::on_task_info_recv,
                                                              this,
                                                              10);
}

bool CartesianRos::validate()
{
    return TaskRos::validate() &&
            _Tref_recv &&
            _vref_recv &&
            !_info.distal_link.empty();
}

void CartesianRos::enableOnlineTrajectoryGeneration()
{
}

bool CartesianRos::isSubtaskLocal() const
{
    return get_task_info().use_local_subtasks;
}

void CartesianRos::getVelocityLimits(double & max_vel_lin,
                                     double & max_vel_ang) const
{
    auto info = get_task_info();
    max_vel_lin = info.max_vel_lin;
    max_vel_ang = info.max_vel_ang;
}

void CartesianRos::getAccelerationLimits(double & max_acc_lin,
                                         double & max_acc_ang) const
{
    auto info = get_task_info();
    max_acc_lin = info.max_acc_lin;
    max_acc_ang = info.max_acc_ang;
}

void CartesianRos::setVelocityLimits(double max_vel_lin,
                                     double max_vel_ang)
{
    auto req = std::make_shared<SetSafetyLimits::Request>();

    req->max_vel_lin = max_vel_lin;
    req->max_vel_ang = max_vel_ang;
    req->max_acc_lin = -1.0;
    req->max_acc_ang = -1.0;

    auto cli = _set_safety_lims_cli;

    auto fut = cli->async_send_request(req);

    if(rclcpp::spin_until_future_complete(_node, fut, 1s) == rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_INFO_STREAM(_node->get_logger(), fut.get()->message);

        if(fut.get()->success)
        {
            return;
        }

        throw std::runtime_error(fmt::format("service '{}' returned false: {}",
                                             cli->get_service_name(),
                                             fut.get()->message));
    }
    else
    {
        throw std::runtime_error(fmt::format("unable to call service '{}'",
                                             cli->get_service_name()));
    }
}

void CartesianRos::setAccelerationLimits(double max_acc_lin,
                                         double max_acc_ang)
{
    auto req = std::make_shared<SetSafetyLimits::Request>();

    req->max_vel_lin = -1.0;
    req->max_vel_ang = -1.0;
    req->max_acc_lin = max_acc_lin;
    req->max_acc_ang = max_acc_ang;

    auto cli = _set_safety_lims_cli;

    auto fut = cli->async_send_request(req);

    if(rclcpp::spin_until_future_complete(_node, fut, 1s) == rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_INFO_STREAM(_node->get_logger(), fut.get()->message);

        if(fut.get()->success)
        {
            return;
        }

        throw std::runtime_error(fmt::format("service '{}' returned false: {}",
                                             cli->get_service_name(),
                                             fut.get()->message));
    }
    else
    {
        throw std::runtime_error(fmt::format("unable to call service '{}'",
                                             cli->get_service_name()));
    }
}

State CartesianRos::getTaskState() const
{
    return StringToEnum<State>(get_task_info().state);
}

const std::string & CartesianRos::getBaseLink() const
{
    _base_link = get_task_info().base_link;
    return _base_link;
}

bool CartesianRos::setBaseLink(const std::string & new_base_link)
{
    auto req = std::make_shared<SetBaseLink::Request>();

    req->base_link = new_base_link;

    auto cli = _set_base_link_cli;

    auto fut = cli->async_send_request(req);

    if(rclcpp::spin_until_future_complete(_node, fut, 1s) == rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_INFO_STREAM(_node->get_logger(), fut.get()->message);

        if(fut.get()->success)
        {
            return true;
        }

        throw std::runtime_error(fmt::format("service '{}' returned false: {}",
                                             cli->get_service_name(),
                                             fut.get()->message));
    }
    else
    {
        throw std::runtime_error(fmt::format("unable to call service '{}'",
                                             cli->get_service_name()));
    }
}

const std::string & CartesianRos::getDistalLink() const
{
    _distal_link = get_task_info().distal_link;
    return _distal_link;
}

ControlType CartesianRos::getControlMode() const
{
    return StringToEnum<ControlType>(get_task_info().control_mode);
}

bool CartesianRos::setControlMode(const ControlType & value)
{
    auto req = std::make_shared<SetControlMode::Request>();

    req->ctrl_mode = EnumToString(value);

    auto cli = _set_ctrl_mode_cli;

    auto fut = cli->async_send_request(req);

    if(rclcpp::spin_until_future_complete(_node, fut, 1s) == rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_INFO_STREAM(_node->get_logger(), fut.get()->message);

        if(fut.get()->success)
        {
            return true;
        }

        throw std::runtime_error(fmt::format("service '{}' returned false: {}",
                                             cli->get_service_name(),
                                             fut.get()->message));
    }
    else
    {
        throw std::runtime_error(fmt::format("unable to call service '{}'",
                                             cli->get_service_name()));
    }
}

bool CartesianRos::getCurrentPose(Eigen::Affine3d& base_T_ee) const
{
    throw std::runtime_error(fmt::format("Unsupported function '{}'",
                                         __PRETTY_FUNCTION__));
}

bool CartesianRos::getPoseReference(Eigen::Affine3d& base_T_ref,
                                    Eigen::Vector6d* base_vel_ref,
                                    Eigen::Vector6d* base_acc_ref) const
{
    base_T_ref = _Tref;
    if(base_vel_ref) *base_vel_ref = _vref;
    if(base_acc_ref) base_acc_ref->setZero();
    return true;
}

bool CartesianRos::getPoseReferenceRaw(Eigen::Affine3d& base_T_ref, Eigen::Vector6d * base_vel_ref, Eigen::Vector6d * base_acc_ref) const
{
    throw std::runtime_error(fmt::format("Unsupported function '{}'",
                                         __PRETTY_FUNCTION__));
}

bool CartesianRos::setPoseReference(const Eigen::Affine3d& base_T_ref)
{
    PoseStamped msg;
    msg.header.stamp = _node->now();
    msg.header.frame_id = "";
    msg.pose = tf2::toMsg(base_T_ref);
    _pose_ref_pub->publish(msg);

    return true;
}

bool CartesianRos::setPoseReferenceRaw(const Eigen::Affine3d& base_T_ref)
{
    throw std::runtime_error(fmt::format("Unsupported function '{}'",
                                         __PRETTY_FUNCTION__));
}

bool CartesianRos::setVelocityReference(const Eigen::Vector6d& base_vel_ref)
{
    return setVelocityReference(base_vel_ref, "");
}

bool CartesianRos::setAccelerationReference(const Eigen::Vector6d &base_acc_ref)
{
    throw std::runtime_error(fmt::format("Unsupported function '{}'",
                                         __PRETTY_FUNCTION__));
}

bool CartesianRos::setVelocityReference(const Eigen::Vector6d& base_vel_ref,
                                        const std::string& base_frame)
{
    TwistStamped msg;
    msg.header.frame_id = base_frame;
    msg.header.stamp = _node->now();
    msg.twist = tf2::toMsg(base_vel_ref);
    _vel_ref_pub->publish(msg);

    return true;
}

bool CartesianRos::getPoseTarget(Eigen::Affine3d& base_T_ref) const
{
    throw std::runtime_error(fmt::format("unsupported function '{}'",
                                         __PRETTY_FUNCTION__));
}

bool CartesianRos::setPoseTarget(const Eigen::Affine3d& base_T_ref,
                                 double time)
{
    Trajectory::WayPoint wp;
    wp.time = time;
    wp.frame = base_T_ref;

    return setWayPoints({wp});
}

int CartesianRos::getCurrentSegmentId() const
{
    return _current_segment_idx;
}

bool CartesianRos::setWayPoints(const Trajectory::WayPointVector& way_points)
{
    return setWayPoints(way_points, false);
}

void CartesianRos::abort()
{
    auto fut = _action_cli->async_cancel_all_goals();

    if(rclcpp::spin_until_future_complete(_node, fut, 1s) == rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_INFO(_node->get_logger(), "abort succeeded");
    }
    else
    {
        RCLCPP_ERROR(_node->get_logger(), "abort failed");
    }
}

bool CartesianRos::waitReachCompleted(double timeout)
{
    auto ret = rclcpp::spin_until_future_complete(_node,
                                                  _action_future,
                                                  std::chrono::duration<double>(timeout));

    return (ret == rclcpp::FutureReturnCode::SUCCESS);
}

bool CartesianRos::setWayPoints(const Trajectory::WayPointVector& way_points,
                                bool incremental)
{
    if(!_action_cli->action_server_is_ready())
    {
        RCLCPP_ERROR(_node->get_logger(),
                     "server not ready");

        return false;
    }

    ReachPose::Goal goal;

    goal.incremental = incremental;

    for(const auto& f : way_points)
    {
        if(goal.time.size() > 0 && f.time <= goal.time.back())
        {
            RCLCPP_ERROR(_node->get_logger(),
                         "time vector must be monotonically increasing");

            return false;
        }

        auto frame = tf2::toMsg(f.frame);

        goal.frames.push_back(frame);

        goal.time.push_back(f.time);
    }

    auto send_goal_options = rclcpp_action::Client<ReachPose>::SendGoalOptions();

    using namespace std::placeholders;

    send_goal_options.goal_response_callback =
        std::bind(&CartesianRos::on_action_active, this, _1);

    send_goal_options.feedback_callback =
        std::bind(&CartesianRos::on_reach_feedback_recv, this, _1, _2);

    send_goal_options.result_callback =
        std::bind(&CartesianRos::on_action_done, this, _1);

    _action_future = _action_cli->async_send_goal(goal, send_goal_options);

    return true;
}

GetCartesianTaskInfo::Response CartesianRos::get_task_info() const
{
    if(asyncMode())
    {
        GetCartesianTaskInfo::Response res;
        res.base_link         = _info.base_link        ;
        res.distal_link       = _info.distal_link      ;
        res.control_mode      = _info.control_mode     ;
        res.state             = _info.state            ;
        res.use_local_subtasks = _info.use_local_subtasks;
        res.max_vel_lin       = _info.max_vel_lin      ;
        res.max_vel_ang       = _info.max_vel_ang      ;
        res.max_acc_lin       = _info.max_acc_lin      ;
        res.max_acc_ang       = _info.max_acc_ang      ;

        return res;
    }

    auto req = std::make_shared<GetCartesianTaskInfo::Request>();

    auto cli = _cart_info_cli;

    auto fut = cli->async_send_request(req);

    if(rclcpp::spin_until_future_complete(_node, fut, 1s) == rclcpp::FutureReturnCode::SUCCESS)
    {
        return *fut.get();
    }
    else
    {
        throw std::runtime_error(fmt::format("unable to call service '{}'",
                                             cli->get_service_name()));
    }
}

void CartesianRos::on_reach_feedback_recv(ActionGoalHandle::SharedPtr goal_handle,
                                          const std::shared_ptr<const ReachPose::Feedback> fb)
{
    _current_segment_idx = fb->current_segment_id;
}

void CartesianRos::on_action_active(ActionGoalHandle::SharedPtr goal_handle)
{
    RCLCPP_INFO(_node->get_logger(),
                "reach action for task '%s' has become active",
                getName().c_str());

    _action_goal_handle = goal_handle;
}

void CartesianRos::on_action_done(const ActionGoalHandle::WrappedResult& result)
{
    RCLCPP_INFO(_node->get_logger(),
                "reach action for task '%s' has been completed",
                getName().c_str());

    _current_segment_idx = -1;

    _action_goal_handle.reset();
}

void CartesianRos::on_task_info_recv(CartesianTaskInfo::ConstSharedPtr msg)
{
    _info = *msg;
}


void CartesianRos::registerObserver(TaskObserver::WeakPtr obs)
{
    if(auto cobs = std::dynamic_pointer_cast<CartesianTaskObserver>(obs.lock()))
    {
        _observers.push_back(cobs);
    }
}

void CartesianRos::notifyTaskChanged(const std::string & message)
{
    TaskRos::notifyTaskChanged(message);

    if(message == "BaseLink")
    {
        NOTIFY_OBSERVERS(BaseLink);
    }
    else if(message == "ControlMode")
    {
        NOTIFY_OBSERVERS(ControlMode);
    }
}
