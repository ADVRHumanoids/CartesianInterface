#include "ros/server_api/CartesianRos.h"

#include <eigen_conversions/eigen_msg.h>

#include <cartesian_interface/CartesianInterface.h>

#include "fmt/format.h"

using namespace XBot::Cartesian;
using namespace XBot::Cartesian::ServerApi;

namespace  {

geometry_msgs::Pose get_normalized_pose(const geometry_msgs::Pose& pose)
{
    Eigen::Vector4d coeff(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);

    if(coeff.squaredNorm() < 1e-6)
    {
        coeff[0] = 1.0;
    }

    coeff /= coeff.norm();

    geometry_msgs::Pose norm_pose = pose;
    norm_pose.orientation.w = coeff[0];
    norm_pose.orientation.x = coeff[1];
    norm_pose.orientation.y = coeff[2];
    norm_pose.orientation.z = coeff[3];

    return norm_pose;

}

}

CartesianRos::CartesianRos(CartesianTask::Ptr cart_task,
                           ModelInterface::ConstPtr model):
    TaskRos(cart_task, model)
{
    registerType("Cartesian");

    _cart = cart_task;

    if(!_cart)
    {
        throw std::runtime_error("Provided task does not have expected type 'CartesianTask'");
    }

    _reach_action_manager.reset(new ReachActionManager(_ctx.nh(), task_name, _cart));

    _pose_ref_pub = _ctx.nh().advertise<geometry_msgs::PoseStamped>(_task->getName() + "/current_reference", 1);

    _vel_ref_pub = _ctx.nh().advertise<geometry_msgs::TwistStamped>(_task->getName() + "/current_velocity_reference", 1);

    _pose_ref_sub = _ctx.nh().subscribe(_task->getName() + "/reference", 1,
                                        &CartesianRos::online_position_reference_cb,
                                        this);

    _vel_ref_sub = _ctx.nh().subscribe(_task->getName() + "/velocity_reference", 1,
                                       &CartesianRos::online_velocity_reference_cb,
                                       this);

    _set_base_link_srv = _ctx.nh().advertiseService(_task->getName() + "/set_base_link",
                                                    &CartesianRos::set_base_link_cb, this);

    _set_ctrl_srv = _ctx.nh().advertiseService(_task->getName() + "/set_control_mode",
                                               &CartesianRos::set_control_mode_cb, this);

    _set_safety_srv = _ctx.nh().advertiseService(_task->getName() + "/set_safety_limits",
                                                 &CartesianRos::set_safety_lims_cb, this);

    _get_info_srv = _ctx.nh().advertiseService(_task->getName() + "/get_cartesian_task_properties",
                                               &CartesianRos::get_task_info_cb, this);

}

void CartesianRos::run(ros::Time time)
{
    TaskRos::run(time);

    _reach_action_manager->run();

    publish_ref(time);
}

void CartesianRos::publish_ref(ros::Time time)
{
    /* Pose reference */
    geometry_msgs::PoseStamped pose_msg;

    pose_msg.header.stamp = time;
    pose_msg.header.frame_id = _ctx.tf_prefix_slash() + _cart->getBaseLink();

    Eigen::Affine3d base_T_ee;
    Eigen::Vector6d vel;
    _cart->getPoseReference(base_T_ee, &vel);
    tf::poseEigenToMsg(base_T_ee, pose_msg.pose);

    _pose_ref_pub.publish(pose_msg);


    /* Velocity reference */
    geometry_msgs::TwistStamped vel_msg;

    vel_msg.header = pose_msg.header;
    tf::twistEigenToMsg(vel, vel_msg.twist);

    _vel_ref_pub.publish(vel_msg);


}

void CartesianRos::online_position_reference_cb(geometry_msgs::PoseStampedConstPtr msg)
{
    Eigen::Affine3d T;
    tf::poseMsgToEigen(msg->pose, T);
    _cart->setPoseReference(T);
}

void CartesianRos::online_velocity_reference_cb(geometry_msgs::TwistStampedConstPtr msg)
{
    Eigen::Vector6d vel;
    tf::twistMsgToEigen(msg->twist, vel);

    Eigen::Matrix3d b_R_f;
    b_R_f.setIdentity();

    if(msg->header.frame_id == "world")
    {
        _model->getOrientation(_cart->getBaseLink(), b_R_f);
        b_R_f.transposeInPlace();
    }
    else if(msg->header.frame_id != "")
    {

        if(!_model->getOrientation(msg->header.frame_id, _cart->getBaseLink(), b_R_f))
        {
            XBot::Logger::error("Unable to set velocity reference for task '%s' (frame_id '%s' undefined)\n",
                                _task->getName().c_str(),
                                msg->header.frame_id.c_str()
                                );

            return;
        }

    }

    vel.head<3>() = b_R_f * vel.head<3>();
    vel.tail<3>() = b_R_f * vel.tail<3>();
    _cart->setVelocityReference(vel);
}

bool CartesianRos::get_task_info_cb(cartesian_interface::GetCartesianTaskInfoRequest&,
                                    cartesian_interface::GetCartesianTaskInfoResponse& res)
{
    res.state = EnumToString(_cart->getTaskState());
    res.base_link = _cart->getBaseLink();
    res.distal_link = _cart->getDistalLink();
    res.control_mode = EnumToString(_cart->getControlMode());
    res.use_local_subtasks = _cart->isSubtaskLocal();

    _cart->getVelocityLimits(res.max_vel_lin,
                             res.max_vel_ang);

    _cart->getAccelerationLimits(res.max_acc_lin,
                                 res.max_acc_ang);

    return true;
}

bool CartesianRos::set_base_link_cb(cartesian_interface::SetBaseLinkRequest & req,
                                    cartesian_interface::SetBaseLinkResponse & res)
{
    auto old_base_link = _cart->getBaseLink();
    res.success = _cart->setBaseLink(req.base_link);

    if(res.success)
    {
        res.message = fmt::format("Successfully changed base link from '{}' to '{}' for task '{}'",
                                  old_base_link, _cart->getBaseLink(), _cart->getName());
    }
    else
    {
        res.message = fmt::format("Unable to change base link from '{}' to '{}' for task '{}'",
                                  old_base_link, req.base_link, _cart->getName());
    }

    return true;
}

bool CartesianRos::set_control_mode_cb(cartesian_interface::SetControlModeRequest & req,
                                       cartesian_interface::SetControlModeResponse & res)
{
    res.success = _cart->setControlMode(StringToEnum<ControlType>(req.ctrl_mode));

    if(res.success)
    {
        res.message = fmt::format("Successfully changed control mode to '{}' for task '{}'",
                                  EnumToString(_cart->getControlMode()),
                                  _cart->getName());
    }
    else
    {
        res.message = fmt::format("Unable to change control mode to '{}' for task '{}'",
                                  req.ctrl_mode,
                                  _cart->getName());
    }

    return true;
}

bool CartesianRos::set_safety_lims_cb(cartesian_interface::SetSafetyLimitsRequest & req,
                                      cartesian_interface::SetSafetyLimitsResponse & res)
{
    double lin, ang;
    _cart->getVelocityLimits(lin, ang);

    if(req.max_vel_lin > 0) lin = req.max_vel_lin;
    if(req.max_vel_ang > 0) ang = req.max_vel_ang;

    _cart->setVelocityLimits(lin, ang);

    res.message = fmt::format("Successfully set velocity limits to {}/{}, ",
                              lin, ang);

    _cart->getAccelerationLimits(lin, ang);

    if(req.max_acc_lin > 0) lin = req.max_acc_lin;
    if(req.max_acc_ang > 0) ang = req.max_acc_ang;

    res.message += fmt::format("acceleration limits to {}/{} (linear/angular)",
                               lin, ang);

    _cart->setAccelerationLimits(lin, ang);

    res.success = true;
    return true;
}

ReachActionManager::ReachActionManager(ros::NodeHandle nh,
                                       std::string task_name,
                                       CartesianTask::Ptr task):
    _action_server(new ActionServer(nh, task_name + "/reach", false)),
    _task(task),
    _state(ReachActionState::IDLE),
    _name(task_name)
{
    _action_server->start();
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
    // wait for new goal to be available,
    // then transit to 'accepted'

    if(_action_server->isNewGoalAvailable())
    {
        Logger::info(Logger::Severity::HIGH,
                     "Received new goal for task '%s'\n", _name.c_str());

        // obtain new goal
        auto goal = _action_server->acceptNewGoal();

        // check consistency
        if(goal->frames.size() != goal->time.size() || goal->frames.size() == 0)
        {
            _action_server->setAborted(cartesian_interface::ReachPoseResult(),
                                       "Time and frame size must match and not be empty");

            Logger::error("Invalid goal received for task '%s' \n", _name.c_str());

            return; // next state is 'idle'
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
            tf::poseMsgToEigen(get_normalized_pose(goal->frames[k]), T_ref);

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
            _action_server->setAborted(cartesian_interface::ReachPoseResult(), "Internal error");
            return; // next state is 'idle'
        }

        // transit to 'accepted'
        _state = ReachActionState::ACCEPTED;
        return;
    }
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
    if(_action_server->isPreemptRequested())
    {
        XBot::Logger::info(XBot::Logger::Severity::HIGH,
                           "Goal for task '%s' canceled by user\n",
                           _name.c_str());

        _task->abort();

        cartesian_interface::ReachPoseResult result;
        Eigen::Affine3d base_T_ee, base_T_ref, ref_T_ee;
        _task->getCurrentPose(base_T_ee);
        _task->getPoseReference(base_T_ref);
        ref_T_ee = base_T_ref.inverse() * base_T_ee;

        tf::poseEigenToMsg(base_T_ref, result.final_frame);
        result.position_error_norm = ref_T_ee.translation().norm();
        result.orientation_error_angle = Eigen::AngleAxisd(ref_T_ee.linear()).angle();

        _action_server->setPreempted(result);
        _state = ReachActionState::COMPLETED; // next state is 'completed'
        return;
    }

    // trajectory ended
    if(_task->getTaskState() == State::Online)
    {

        cartesian_interface::ReachPoseResult result;
        Eigen::Affine3d base_T_ee, base_T_ref, ref_T_ee;
        _task->getCurrentPose(base_T_ee);
        _task->getPoseReference(base_T_ref);
        ref_T_ee = base_T_ref.inverse() * base_T_ee;

        tf::poseEigenToMsg(base_T_ee, result.final_frame);
        result.position_error_norm = ref_T_ee.translation().norm();
        result.orientation_error_angle = Eigen::AngleAxisd(ref_T_ee.linear()).angle();

        XBot::Logger::success(XBot::Logger::Severity::HIGH,
                              "Goal for task '%s' succeded \n"
                              "Final position error norm: %f m \n"
                              "Final orientation error: %f rad \n",
                              _name.c_str(),
                              result.position_error_norm,
                              result.orientation_error_angle);

        _action_server->setSucceeded(result);
        _state = ReachActionState::COMPLETED; // next state is 'completed'
        return;
    }
    else // publish feedback
    {
        cartesian_interface::ReachPoseFeedback feedback;

        Eigen::Affine3d base_T_ref;
        feedback.current_reference.header.stamp = ros::Time::now();
        feedback.current_reference.header.frame_id = _task->getBaseLink();
        _task->getPoseReference(base_T_ref);
        tf::poseEigenToMsg(base_T_ref, feedback.current_reference.pose);

        Eigen::Affine3d base_T_ee;
        feedback.current_pose.header.stamp = ros::Time::now();
        feedback.current_pose.header.frame_id = _task->getBaseLink();
        _task->getCurrentPose(base_T_ee);
        tf::poseEigenToMsg(base_T_ee, feedback.current_pose.pose);

        feedback.current_segment_id = _task->getCurrentSegmentId();

        _action_server->publishFeedback(feedback);
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
