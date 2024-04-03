#include <cartesian_interface/ros/RosServerClass.h>
#include <cartesian_interface/GetTaskListResponse.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/WrenchStamped.h>

using namespace XBot::Cartesian;

namespace  {

geometry_msgs::Pose get_normalized_pose(const geometry_msgs::Pose& pose)
{
    Eigen::Vector4d coeff(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);

    if(coeff.squaredNorm() == 0)
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

RosServerClass::Options::Options():
    tf_prefix("ci"),
    ros_namespace("cartesian"),
    publish_tf(true)
{

}


RosServerClass::RosServerClass(CartesianInterfaceImpl::Ptr intfc,
                               Options opt):
    _ci(intfc),
    _model(intfc->getModel()),
    _opt(opt),
    _nh(opt.ros_namespace),
    _wrench_pubs_inited(false)
{
    _tf_prefix = _opt.tf_prefix;
    _nh.setCallbackQueue(&_cbk_queue);

    _ros_ctx = std::make_shared<RosContext>(_nh, _tf_prefix, intfc->getContext());
    _tf_prefix_slash = _ros_ctx->tf_prefix_slash();


    init_state_broadcasting();
    init_rspub();
    init_task_list_service();
    init_reset_world_service();
    init_heartbeat_pub();
    init_load_ros_task_api();

}

void RosServerClass::init_state_broadcasting()
{
    _solution_pub = _nh.advertise<sensor_msgs::JointState>("solution", 1);
}


void RosServerClass::init_rspub()
{
    _rspub.reset(new RsPub(_model));

    ros::NodeHandle base_nh;
    if(!base_nh.hasParam("robot_description"))
    {
        base_nh.setParam("robot_description", _model->getUrdfString());
    }
    if(!base_nh.hasParam("robot_description_semantic"))
    {
        base_nh.setParam("robot_description_semantic", _model->getSrdfString());
    }
    
    _com_pub = _nh.advertise<geometry_msgs::PointStamped>("com_position", 1);
}

void RosServerClass::publish_ref_tf(ros::Time time)
{
    /* Publish TF */
    _rspub->publishTransforms(time, _tf_prefix);
    
    /* Publish CoM position */
    Eigen::Vector3d com = _model->getCOM();
    
    geometry_msgs::PointStamped com_msg;
    tf::pointEigenToMsg(com, com_msg.point);
    com_msg.header.frame_id = _tf_prefix_slash + "world";
    com_msg.header.stamp = time;
    _com_pub.publish(com_msg);
    
    /* Publish CoM tf */
    Eigen::Affine3d w_T_com;
    if(_model->isFloatingBase())
    {
        _model->getFloatingBasePose(w_T_com);
    }
    else
    {
        w_T_com.setIdentity();
    }
    
    w_T_com.translation() = com;
    
    tf::Transform transform;
    tf::transformEigenToTF(w_T_com, transform);

    _tf_broadcaster.sendTransform(tf::StampedTransform(transform,
                                                       time,
                                                       _tf_prefix_slash + "world",
                                                       _tf_prefix_slash + "com"));

}

void RosServerClass::heartbeat_cb(const ros::TimerEvent & ev)
{
    _heartbeat_pub.publish(std_msgs::Empty());

    //    publish_task_info(); TBD
}

void XBot::Cartesian::RosServerClass::publish_solution(ros::Time time)
{
    sensor_msgs::JointState msg;
    std::vector<geometry_msgs::WrenchStamped> w_msg;
    Eigen::VectorXd _sol_q, _sol_qdot, _sol_tau;
    auto solution = _ci->getSolution();

    if(!_wrench_pubs_inited)
    {
        for(auto& p : solution)
        {
            if(p.first.find("force_") != 0)
            {
                continue;
            }
            _wrench_pubs.push_back(_nh.advertise<geometry_msgs::WrenchStamped>(p.first, 1, true));
        }
        _wrench_pubs_inited = true;
    }

    bool no_subsribers = true;

    if(_solution_pub.getNumSubscribers() != 0)
    {
        no_subsribers = false;
    }

    for(auto& pub : _wrench_pubs)
    {
        if(pub.getNumSubscribers() != 0)
            no_subsribers = false;
    }

    if(no_subsribers)
        return;
    
    _model->getJointPosition(_sol_q);
    _model->getJointVelocity(_sol_qdot);
    _model->getJointEffort(_sol_tau);

    // apply log map to retrieve the motion representation of q
    // this has size = nv
    _sol_q = _model->difference(_sol_q, _model->getNeutralQ());
    
    // to deal with non-euclidean joints, we will publish the
    // log map of q, i.e. the motion that brings the robot to q
    // when applied for unit time starting from q0

    for(auto& p : solution)
    {
        if(p.first.find("force_") != 0)
        {
            continue;
        }

        geometry_msgs::WrenchStamped w;
        w.header.stamp = time;
        auto frame = p.first.substr(6); // removes "force_"

        Eigen::Affine3d w_T_f = _model->getPose(frame);
        Eigen::Vector6d ww = p.second;
        //in local frame
        ww.segment(0,3) = w_T_f.linear().inverse() * ww.segment(0,3);
        ww.segment(3,3) = w_T_f.linear().inverse() * ww.segment(3,3);

        ww.segment(0,3) = ww.segment(0,3)/ww.segment(0,3).norm();
        ww.segment(3,3) = ww.segment(3,3)/ww.segment(3,3).norm();

        w.wrench.force.x = ww[0];
        w.wrench.force.y = ww[1];
        w.wrench.force.z = ww[2];
        w.wrench.torque.x = ww[3];
        w.wrench.torque.y = ww[4];
        w.wrench.torque.z = ww[5];

        w.header.frame_id = "ci/" + frame;

        w_msg.push_back(w);
    }


    for(unsigned int i = 0; i < _wrench_pubs.size(); ++i)
        _wrench_pubs[i].publish(w_msg[i]);

    msg.header.stamp = time;
    msg.name.reserve(_model->getNv());
    msg.position.reserve(_model->getNv());
    msg.velocity.reserve(_model->getNv());
    int i = 0;
    for(const std::string& jname : _model->getVNames())
    {
        msg.name.push_back(jname);
        msg.position.push_back(_sol_q[i]);
        msg.velocity.push_back(_sol_qdot[i]);
        msg.effort.push_back(_sol_tau[i]);
        i++;
    }
    
    _solution_pub.publish(msg);
}


void RosServerClass::run()
{
    _cbk_queue.callAvailable();
    
    ros::Time now = ros::Time::now();

    if(_opt.publish_tf)
    {
        publish_ref_tf(now);
//        publish_world_tf(now);
    }

    publish_solution(now);

    for(auto t : _ros_tasks)
    {
        t->run(now);
    }
}


void RosServerClass::publish_world_tf(ros::Time time)
{
    /* Publish world odom */
    Eigen::Affine3d w_T_pelvis;
    w_T_pelvis.setIdentity();
    std::string fb_link = "world";
    
    if(_model->isFloatingBase())
    {
        _model->getFloatingBasePose(w_T_pelvis);
        _model->getFloatingBaseLink(fb_link);
    }
    
    tf::Transform transform;
    tf::transformEigenToTF(w_T_pelvis, transform);

    _tf_broadcaster.sendTransform(tf::StampedTransform(transform.inverse(),
                                                       time,
                                                       _tf_prefix_slash + fb_link,
                                                       _tf_prefix_slash + "world"));
    
}


void XBot::Cartesian::RosServerClass::init_task_list_service()
{
    std::string srv_name = "get_task_list";
    _tasklist_srv = _nh.advertiseService(srv_name, &RosServerClass::task_list_cb, this);
}

bool XBot::Cartesian::RosServerClass::task_list_cb(cartesian_interface::GetTaskListRequest& req,
                                                   cartesian_interface::GetTaskListResponse& res)
{
    
    for(const auto& tname : _ci->getTaskList())
    {
        auto t = _ci->getTask(tname);

        res.names.push_back(t->getName());
        res.types.push_back(t->getType());
    }
    
    return true;
}

RosServerClass::~RosServerClass()
{
}

XBot::ModelInterface::ConstPtr RosServerClass::getModel() const
{
    return _model;
}


void XBot::Cartesian::RosServerClass::init_reset_world_service()
{
    _reset_world_srv = _nh.advertiseService("reset_world", &RosServerClass::reset_world_cb, this);
    _reset_base_srv = _nh.advertiseService("reset_base", &RosServerClass::reset_base_cb, this);
}

void RosServerClass::init_heartbeat_pub()
{
    const double timer_period = 0.1;
    _heartbeat_pub = _nh.advertise<std_msgs::Empty>("heartbeat", 1);
    _heartbeat_timer = _nh.createTimer(ros::Duration(timer_period),
                                       &RosServerClass::heartbeat_cb,
                                       this);
}

void RosServerClass::init_load_ros_task_api()
{
    for(auto tname : _ci->getTaskList())
    {
        auto t = _ci->getTask(tname);

        auto task_ros = ServerApi::TaskRos::MakeInstance(t, _ros_ctx);

        _ros_tasks.push_back(task_ros);
    }
}

bool XBot::Cartesian::RosServerClass::reset_world_cb(cartesian_interface::ResetWorldRequest& req,
                                                     cartesian_interface::ResetWorldResponse& res)
{
    Eigen::Affine3d new_world;

    if(req.from_link == "")
    {
        tf::poseMsgToEigen(get_normalized_pose(req.new_world), new_world);
    }
    else
    {
        if(!_model->getPose(req.from_link, new_world))
        {
            res.success = false;
            res.message = "Link " + req.from_link + " undefined. World could not be changed.";
            return true;
        }
    }

    if(_ci->resetWorld(new_world))
    {
        res.success = true;
        res.message = "World was changed successfully.";
    }
    else
    {
        res.success = false;
        res.message = "World could not be changed.";
    }

    return true;
}

bool RosServerClass::reset_base_cb(cartesian_interface::SetTransformRequest& req,
                                   cartesian_interface::SetTransformResponse& res)
{
    Eigen::Affine3d nw_T_base, w_T_nw, w_T_base;
    tf::poseMsgToEigen(get_normalized_pose(req.pose), nw_T_base);

    if(!_model->getFloatingBasePose(w_T_base))
    {
        res.success = false;
        res.message = "model is not floating base";
        return true;
    }

    w_T_nw = w_T_base * nw_T_base.inverse();

    _ci->resetWorld(w_T_nw);

    res.success = true;
    res.message = "successfully set floating base pose to model";
    return true;
}
