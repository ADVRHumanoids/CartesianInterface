#include "CartesianRos.h"

#include <eigen_conversions/eigen_msg.h>

#include <cartesian_interface/CartesianInterface.h>

using namespace XBot::Cartesian;

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

CartesianRos::CartesianRos(RosContext ctx, TaskDescription::Ptr task):
    TaskRos(ctx, task)
{
    _cart = std::dynamic_pointer_cast<CartesianTask>(task);

    if(!_cart)
    {
        throw std::runtime_error("Provided task does not have expected type 'CartesianTask'");
    }

    _reach_action_manager.reset(new ReachActionManager(ctx.nh(), task_name, _cart));
}

void CartesianRos::run(ros::Time time)
{
    TaskRos::run(time);

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

}

bool CartesianRos::get_task_info_cb(cartesian_interface::GetTaskInfoRequest & req,
                                    cartesian_interface::GetTaskInfoResponse & res)
{
    res.base_link = _cart->getBaseLink();
    res.control_mode  =  CartesianInterface::ControlTypeAsString(_cart->getControlMode());
    res.task_state  =  CartesianInterface::StateAsString(_cart->getTaskState());
    res.distal_link = _cart->getDistalLink();
    res.type = _cart->getType();
    res.name = _cart->getName();
    return true;
}

bool CartesianRos::set_task_info_cb(cartesian_interface::SetTaskInfoRequest & req,
                                    cartesian_interface::SetTaskInfoResponse & res)
{
    std::string new_base_link = req.base_link;
    if(new_base_link == "world_odom")
    {
        new_base_link = "world";
    }

    if(new_base_link != "" && _cart->setBaseLink(new_base_link))
    {
        res.message = "Successfully set base link of task " + task_name + " to " + new_base_link;
        res.success = true;
    }
    else if(new_base_link != "")
    {
        res.message = "Unable to set base link of task " + task_name + " to " + new_base_link;
        res.success = false;
    }


    if(req.control_mode != "" && _cart->setControlMode(CartesianInterface::ControlTypeFromString(req.control_mode)))
    {
        res.message = "Successfully set control mode of task " + task_name + " to " + req.control_mode;
        res.success = true;
    }
    else if(req.control_mode != "")
    {
        res.message = "Unable to set control mode of task " + task_name + " to " + req.control_mode;
        res.success = false;
    }


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

void ReachActionManager::run_state_idle()
{
    // wait for new goal to be available,
    // then transit to 'accepted'

    if(_action_server->isNewGoalAvailable())
    {
        Logger::info(Logger::Severity::HIGH,
                     "Accepted new goal for task '%s'\n", _name.c_str());

        // obtain new goal
        auto goal = _action_server->acceptNewGoal();

        // check consistency
        if(goal->frames.size() != goal->time.size())
        {
            _action_server->setAborted(cartesian_interface::ReachPoseResult(),
                                       "Time and frame size must match");
            return; // next state is 'idle'
        }

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

