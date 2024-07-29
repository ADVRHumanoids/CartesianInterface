#include "ros/server_api/CartesianRos.h"

#include <cartesian_interface/CartesianInterface.h>

#include "fmt/format.h"

#include "../utils/RosUtils.h"

#include <tf2_eigen/tf2_eigen.hpp>

#include <xbot2_interface/logger.h>

using namespace XBot::Cartesian;
using namespace XBot::Cartesian::ServerApi;
using XBot::Logger;

namespace  {

geometry_msgs::msg::Pose get_normalized_pose(const geometry_msgs::msg::Pose& pose)
{
    Eigen::Vector4d coeff(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);

    if(coeff.squaredNorm() < 1e-6)
    {
        coeff[0] = 1.0;
    }

    coeff /= coeff.norm();

    geometry_msgs::msg::Pose norm_pose = pose;
    norm_pose.orientation.w = coeff[0];
    norm_pose.orientation.x = coeff[1];
    norm_pose.orientation.y = coeff[2];
    norm_pose.orientation.z = coeff[3];

    return norm_pose;

}

}

CartesianRos::CartesianRos(CartesianTask::Ptr cart_task,
                           RosContext::Ptr context):
    TaskRos(cart_task, context)
{
    registerType("Cartesian");

    _cart = cart_task;

    if(!_cart)
    {
        throw std::runtime_error("Provided task does not have expected type 'CartesianTask'");
    }

    auto n = _ctx->node();

    _reach_action_manager = std::make_unique<ReachActionManager>(n, task_name, _cart);

    _pose_ref_pub = n->create_publisher<PoseStamped>(
                _task->getName() + "/current_reference", 1
                );

    _vel_ref_pub = n->create_publisher<TwistStamped>(
                _task->getName() + "/current_velocity_reference", 1
                );

    _acc_ref_pub = n->create_publisher<TwistStamped>(
                _task->getName() + "/current_acceleration_reference", 1
                );

    _pose_ref_sub = ::create_subscription<PoseStamped>(n,
                                         _task->getName() + "/reference",
                                         &CartesianRos::online_position_reference_cb,
                                         this);

    _task_info_pub = n->create_publisher<CartesianTaskInfo>(
                _task->getName() + "/cartesian_task_properties", 1
                );

    _vel_ref_sub = ::create_subscription<TwistStamped>(n,_task->getName() + "/velocity_reference",
                                        &CartesianRos::online_velocity_reference_cb,
                                        this);

    _set_base_link_srv = ::create_service<SetBaseLink>(n,
                                                       _task->getName() + "/set_base_link",
                                                       &CartesianRos::set_base_link_cb,
                                                       this);

    _set_ctrl_srv = ::create_service<SetControlMode>(n,
                                                     _task->getName() + "/set_control_mode",
                                                     &CartesianRos::set_control_mode_cb,
                                                     this);

    _set_safety_srv = ::create_service<SetSafetyLimits>(n,
                                                        _task->getName() + "/set_safety_limits",
                                                        &CartesianRos::set_safety_lims_cb,
                                                        this);

    _get_info_srv = ::create_service<GetCartesianTaskInfo>(n,
                                                           _task->getName()
                                                               + "/get_cartesian_task_properties",
                                                           &CartesianRos::get_task_info_cb,
                                                           this);
}

void CartesianRos::run(rclcpp::Time time)
{
    TaskRos::run(time);

    _reach_action_manager->run();

    publish_ref(time);

    publish_task_info();
}

void CartesianRos::publish_ref(rclcpp::Time time)
{
    /* Pose reference */
    geometry_msgs::msg::PoseStamped pose_msg;

    pose_msg.header.stamp = time;
    pose_msg.header.frame_id = _ctx->tf_prefix_slash() + _cart->getBaseLink();

    Eigen::Affine3d base_T_ee;
    Eigen::Vector6d vel, acc;
    _cart->getPoseReference(base_T_ee, &vel, &acc);
    pose_msg.pose = tf2::toMsg(base_T_ee);

    _pose_ref_pub->publish(pose_msg);


    /* Velocity reference */
    geometry_msgs::msg::TwistStamped vel_msg;

    vel_msg.header = pose_msg.header;
    vel_msg.twist = tf2::toMsg(vel);

    _vel_ref_pub->publish(vel_msg);

    /* Acceleration reference */
    geometry_msgs::msg::TwistStamped acc_msg;

    acc_msg.header = pose_msg.header;
    acc_msg.twist = tf2::toMsg(acc);

    _acc_ref_pub->publish(acc_msg);

}

void CartesianRos::publish_task_info()
{
    auto info_req = std::make_shared<GetCartesianTaskInfo::Request>();
    auto info_res = std::make_shared<GetCartesianTaskInfo::Response>();
    get_task_info_cb(info_req, info_res);

    CartesianTaskInfo msg;
    msg.base_link = info_res->base_link;
    msg.control_mode = info_res->control_mode;
    msg.distal_link = info_res->distal_link;
    msg.max_acc_ang = info_res->max_acc_ang;
    msg.max_acc_lin = info_res->max_acc_lin;
    msg.max_vel_ang = info_res->max_vel_ang;
    msg.max_vel_lin = info_res->max_vel_lin;
    msg.state = info_res->state;
    msg.use_local_subtasks = info_res->use_local_subtasks;

    _task_info_pub->publish(msg);

}

void CartesianRos::online_position_reference_cb(PoseStamped::ConstSharedPtr msg)
{
    Eigen::Affine3d T;
    tf2::fromMsg(msg->pose, T);
    _cart->setPoseReference(T);
}

void CartesianRos::online_velocity_reference_cb(TwistStamped::ConstSharedPtr msg)
{
    Eigen::Vector6d vel;
    tf2::fromMsg(msg->twist, vel);

    Eigen::Matrix3d b_R_f;
    b_R_f.setIdentity();

    if(msg->header.frame_id == "world")
    {
        _model->getOrientation(_cart->getBaseLink(), b_R_f);
        b_R_f.transposeInPlace();
    }
    else if(msg->header.frame_id != "")
    {

        if(!_model->getLinkId(msg->header.frame_id) < 0)
        {
            XBot::Logger::error("Unable to set velocity reference for task '%s' (frame_id '%s' undefined)\n",
                                _task->getName().c_str(),
                                msg->header.frame_id.c_str()
                                );

            return;
        }

        b_R_f = _model->getPose(msg->header.frame_id, _cart->getBaseLink()).linear();

    }

    vel.head<3>() = b_R_f * vel.head<3>();
    vel.tail<3>() = b_R_f * vel.tail<3>();
    _cart->setVelocityReference(vel);
}

bool CartesianRos::get_task_info_cb(GetCartesianTaskInfo::Request::ConstSharedPtr req,
                                    GetCartesianTaskInfo::Response::SharedPtr res)
{
    res->state = EnumToString(_cart->getTaskState());
    res->base_link = _cart->getBaseLink();
    res->distal_link = _cart->getDistalLink();
    res->control_mode = EnumToString(_cart->getControlMode());
    res->use_local_subtasks = _cart->isSubtaskLocal();

    _cart->getVelocityLimits(res->max_vel_lin,
                             res->max_vel_ang);

    _cart->getAccelerationLimits(res->max_acc_lin,
                                 res->max_acc_ang);

    return true;
}

bool CartesianRos::set_base_link_cb(SetBaseLink::Request::ConstSharedPtr req,
                                    SetBaseLink::Response::SharedPtr res)
{
    auto old_base_link = _cart->getBaseLink();
    res->success = _cart->setBaseLink(req->base_link);

    if(res->success)
    {
        res->message = fmt::format("Successfully changed base link from '{}' to '{}' for task '{}'",
                                  old_base_link, _cart->getBaseLink(), _cart->getName());
    }
    else
    {
        res->message = fmt::format("Unable to change base link from '{}' to '{}' for task '{}'",
                                  old_base_link, req->base_link, _cart->getName());
    }

    return true;
}

bool CartesianRos::set_control_mode_cb(SetControlMode::Request::ConstSharedPtr req,
                                       SetControlMode::Response::SharedPtr res)
{
    res->success = _cart->setControlMode(StringToEnum<ControlType>(req->ctrl_mode));

    if(res->success)
    {
        res->message = fmt::format("Successfully changed control mode to '{}' for task '{}'",
                                  EnumToString(_cart->getControlMode()),
                                  _cart->getName());
    }
    else
    {
        res->message = fmt::format("Unable to change control mode to '{}' for task '{}'",
                                  req->ctrl_mode,
                                  _cart->getName());
    }

    return true;
}

bool CartesianRos::set_safety_lims_cb(SetSafetyLimits::Request::ConstSharedPtr req,
                                      SetSafetyLimits::Response::SharedPtr res)
{
    double lin, ang;
    _cart->getVelocityLimits(lin, ang);

    if(req->max_vel_lin > 0) lin = req->max_vel_lin;
    if(req->max_vel_ang > 0) ang = req->max_vel_ang;

    _cart->setVelocityLimits(lin, ang);

    res->message = fmt::format("Successfully set velocity limits to {}/{}, ",
                              lin, ang);

    _cart->getAccelerationLimits(lin, ang);

    if(req->max_acc_lin > 0) lin = req->max_acc_lin;
    if(req->max_acc_ang > 0) ang = req->max_acc_ang;

    res->message += fmt::format("acceleration limits to {}/{} (linear/angular)",
                               lin, ang);

    _cart->setAccelerationLimits(lin, ang);

    res->success = true;
    return true;
}

ReachActionManager::ReachActionManager(rclcpp::Node::SharedPtr node,
                                       std::string task_name,
                                       CartesianTask::Ptr task):
    _task(task),
    _state(ReachActionState::IDLE),
    _name(task_name)
{
    using namespace std::placeholders;

    _action_server = rclcpp_action::create_server<ReachPose>(
        node,
        task_name + "/reach",
        std::bind(&ReachActionManager::handle_goal, this, _1, _2),
        std::bind(&ReachActionManager::handle_cancel, this, _1),
        std::bind(&ReachActionManager::handle_accepted, this, _1)
        );
}

rclcpp_action::GoalResponse ReachActionManager::handle_goal(
    const rclcpp_action::GoalUUID &uuid,
    std::shared_ptr<const ReachPose::Goal> goal)
{
    Logger::info(Logger::Severity::HIGH,
                 "Received new goal for task '%s'\n", _name.c_str());

    // check consistency
    if(goal->frames.size() != goal->time.size() || goal->frames.size() == 0)
    {
        Logger::error("Invalid goal received for task '%s' \n", _name.c_str());

        return rclcpp_action::GoalResponse::REJECT;
    }

    Logger::info(Logger::Severity::HIGH,
                 "Accepted new goal for task '%s'\n", _name.c_str());

    // get current state for task (note: should it be getPoseReference instead?)
    Eigen::Affine3d base_T_ee;
    _task->getCurrentPose(base_T_ee);

    // fill waypoint vector
    Trajectory::WayPointVector waypoints;

    for(int k = 0; k < goal->frames.size(); k++)
    {
        Eigen::Affine3d T_ref;
        tf2::fromMsg(get_normalized_pose(goal->frames[k]), T_ref);

        if(goal->incremental)
        {
            T_ref.linear() = base_T_ee.linear() *  T_ref.linear();
            T_ref.translation() = base_T_ee.translation() +  T_ref.translation();
        }

        Trajectory::WayPoint wp;
        wp.frame = T_ref;
        wp.time = goal->time[k];
        waypoints.push_back(wp);

    }

    // send waypoints to cartesian ifc
    if(!_task->setWayPoints(waypoints))
    {
        return rclcpp_action::GoalResponse::REJECT; // next state is 'idle'
    }

    // initialize feedback
    _fb = std::make_shared<ReachPose::Feedback>();

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse ReachActionManager::handle_cancel(
    const std::shared_ptr<GoalHandle> goal_handle)
{
    RCLCPP_INFO(_node->get_logger(), "Received request to cancel goal");
    _goal_handle.reset();
    return rclcpp_action::CancelResponse::ACCEPT;
}

void ReachActionManager::handle_accepted(const std::shared_ptr<GoalHandle> goal_handle)
{
    _goal_handle = goal_handle;
    _state = ReachActionState::ACCEPTED;
}

void ReachActionManager::run()
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

ReachActionManager::~ReachActionManager()
{
}

void ReachActionManager::run_state_idle()
{

}

void ReachActionManager::run_state_accepted()
{
    // wait till cartesian ifc switches state to 'reaching'
    if(_task->getTaskState() == State::Reaching)
    {
        Logger::info(Logger::Severity::HIGH,
                     "Reaching started for task '%s'\n", _name.c_str());

        _state = ReachActionState::RUNNING; // next state is 'running'
        return;
    }
}

void ReachActionManager::run_state_running()
{
    // manage preemption
    if(_goal_handle->is_canceling())
    {
        XBot::Logger::info(XBot::Logger::Severity::HIGH,
                           "Goal for task '%s' canceled by user\n",
                           _name.c_str());

        _task->abort();

        auto result = std::make_shared<ReachPose::Result>();

        Eigen::Affine3d base_T_ee, base_T_ref, ref_T_ee;
        _task->getCurrentPose(base_T_ee);
        _task->getPoseReference(base_T_ref);
        ref_T_ee = base_T_ref.inverse() * base_T_ee;

        result->final_frame = tf2::toMsg(base_T_ee);
        result->position_error_norm = ref_T_ee.translation().norm();
        result->orientation_error_angle = Eigen::AngleAxisd(ref_T_ee.linear()).angle();

        _goal_handle->canceled(result);
        _state = ReachActionState::COMPLETED; // next state is 'completed'
        return;
    }

    // trajectory ended
    if(_task->getTaskState() == State::Online)
    {
        auto result = std::make_shared<ReachPose::Result>();

        Eigen::Affine3d base_T_ee, base_T_ref, ref_T_ee;
        _task->getCurrentPose(base_T_ee);
        _task->getPoseReference(base_T_ref);
        ref_T_ee = base_T_ref.inverse() * base_T_ee;

        result->final_frame = tf2::toMsg(base_T_ee);
        result->position_error_norm = ref_T_ee.translation().norm();
        result->orientation_error_angle = Eigen::AngleAxisd(ref_T_ee.linear()).angle();

        XBot::Logger::success(XBot::Logger::Severity::HIGH,
                              "Goal for task '%s' succeded \n"
                              "Final position error norm: %f m \n"
                              "Final orientation error: %f rad \n",
                              _name.c_str(),
                              result->position_error_norm,
                              result->orientation_error_angle);

        _goal_handle->succeed(result);
        _goal_handle.reset();
        _state = ReachActionState::COMPLETED; // next state is 'completed'
        return;
    }
    else // publish feedback
    {
        Eigen::Affine3d base_T_ref;
        _fb->current_reference.header.stamp = _node->get_clock()->now();
        _fb->current_reference.header.frame_id = _task->getBaseLink();
        _task->getPoseReference(base_T_ref);
        _fb->current_reference.pose = tf2::toMsg(base_T_ref);

        Eigen::Affine3d base_T_ee;
        _fb->current_pose.header.stamp = _fb->current_reference.header.stamp;
        _fb->current_pose.header.frame_id = _task->getBaseLink();
        _task->getCurrentPose(base_T_ee);
        _fb->current_pose.pose = tf2::toMsg(base_T_ee);

        _fb->current_segment_id = _task->getCurrentSegmentId();

        _goal_handle->publish_feedback(_fb);
        return; // next state is 'running'

    }
}

void ReachActionManager::run_state_completed()
{
    XBot::Logger::info(XBot::Logger::Severity::HIGH,
                       "Goal for task '%s' completed\n",
                       _name.c_str());

    _state = ReachActionState::IDLE;
    return;
}

bool XBot::Cartesian::ServerApi::CartesianRos::onBaseLinkChanged()
{
    notifyTaskChanged("BaseLink");
    return true;
}

bool XBot::Cartesian::ServerApi::CartesianRos::onControlModeChanged()
{
    notifyTaskChanged("ControlMode");
    return true;
}
