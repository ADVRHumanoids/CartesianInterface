#include <cartesian_interface/ros/RosServerClass.h>
#include <cartesian_interface/GetTaskListResponse.h>

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
    ros_namespace("cartesian")
{

}

void RosServerClass::init_state_broadcasting()
{
    for(std::string ee_name : _cartesian_interface->getTaskList())
    {
        std::string topic_name = "" + ee_name + "/state";

        ros::Publisher pub = _nh.advertise<geometry_msgs::PoseStamped>(topic_name,
                                                                        1);

        _state_pub.push_back(pub);
    }
    
    _solution_pub = _nh.advertise<sensor_msgs::JointState>("solution", 1);
}


void RosServerClass::init_rspub()
{
    KDL::Tree kdl_tree;
    kdl_parser::treeFromUrdfModel(_model->getUrdf(), kdl_tree);

    _rspub.reset(new RsPub(kdl_tree));

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
    XBot::JointNameMap _joint_name_map;
    _model->getJointPosition(_joint_name_map);
    std::map<std::string, double> _joint_name_std_map;
    
    auto predicate = [](const std::pair<std::string, double>& pair)
    {
        return pair.first.find("VIRTUALJOINT") == std::string::npos;
    };
    
    std::copy_if(_joint_name_map.begin(), _joint_name_map.end(),
                 std::inserter(_joint_name_std_map, _joint_name_std_map.end()),
                 predicate);

    _rspub->publishTransforms(_joint_name_std_map, time, _tf_prefix);
    _rspub->publishFixedTransforms(_tf_prefix, true);
    
    
    /* Publish CoM position */
    Eigen::Vector3d com;
    _model->getCOM(com);
    
    geometry_msgs::PointStamped com_msg;
    tf::pointEigenToMsg(com, com_msg.point);
    com_msg.header.frame_id = _tf_prefix_slash + "world_odom";
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
                                                       _tf_prefix_slash + "world_odom",
                                                       _tf_prefix_slash + "com"));

}



void RosServerClass::online_position_reference_cb(const geometry_msgs::PoseStampedConstPtr& msg,
                                                  const std::string& ee_name)
{

    Eigen::Affine3d T;
    tf::poseMsgToEigen(msg->pose, T);
    
    _cartesian_interface->setPoseReference(ee_name, T);
    
}

void XBot::Cartesian::RosServerClass::online_force_reference_cb(const geometry_msgs::WrenchStampedConstPtr& msg, 
                                                                const std::string& ee_name)
{
    Eigen::Vector6d f;
    tf::wrenchMsgToEigen(msg->wrench, f);
    
    _cartesian_interface->setForceReference(ee_name, f);
}


void XBot::Cartesian::RosServerClass::init_interaction_topics()
{
    for(std::string ee_name : _cartesian_interface->getTaskList())
    {
        if(_cartesian_interface->getTaskInterface(ee_name) != TaskInterface::Interaction)
        {
            continue;
        }
        
        auto icb = std::bind(&RosServerClass::online_impedance_reference_cb, this, std::placeholders::_1, ee_name);

        ros::Subscriber isub = _nh.subscribe<cartesian_interface::Impedance6>(ee_name + "/impedance",
                                                                             1, icb);

        _imp_sub.push_back(isub);
        
        auto fcb = std::bind(&RosServerClass::online_force_reference_cb, this, std::placeholders::_1, ee_name);
        
        ros::Subscriber fsub = _nh.subscribe<geometry_msgs::WrenchStamped>(ee_name + "/force",
                                                                             1, fcb);

        _force_sub.push_back(fsub);
    }
}

void XBot::Cartesian::RosServerClass::init_interaction_srvs()
{
    for(std::string ee_name : _cartesian_interface->getTaskList())
    {
        if(_cartesian_interface->getTaskInterface(ee_name) != TaskInterface::Interaction)
        {
            continue;
        }
        
        std::string srv_name = ee_name + "/get_impedance";

        auto cb = boost::bind(&RosServerClass::get_impedance_cb,
                            this,
                            _1,
                            _2,
                            ee_name);
        
        ros::ServiceServer srv = _nh.advertiseService<cartesian_interface::GetImpedanceRequest,
                                                      cartesian_interface::GetImpedanceResponse>(srv_name, cb);

        _impedance_srv.push_back(srv);
    }
}


void RosServerClass::init_online_vel_topics()
{
    for(std::string ee_name : _cartesian_interface->getTaskList())
    {
        std::string topic_name = ee_name + "/velocity_reference";

        auto cb = std::bind(&RosServerClass::online_velocity_reference_cb, this, std::placeholders::_1, ee_name);

        ros::Subscriber sub = _nh.subscribe<geometry_msgs::TwistStamped>(topic_name,
                                                                        1, cb);

        _vel_sub.push_back(sub);
    }
}

void RosServerClass::online_velocity_reference_cb(const geometry_msgs::TwistStampedConstPtr& msg, 
                                                  const std::string& ee_name)
{
    Eigen::Vector6d vel;
    tf::twistMsgToEigen(msg->twist, vel);
    
    Eigen::Matrix3d b_R_f;
    b_R_f.setIdentity();
    
    if(msg->header.frame_id == "world")
    {
        _model->getOrientation(_cartesian_interface->getBaseLink(ee_name), b_R_f);
        b_R_f.transposeInPlace();
    }
    else if(msg->header.frame_id != "")
    {
        
        if(!_model->getOrientation(msg->header.frame_id, _cartesian_interface->getBaseLink(ee_name), b_R_f))
        {
            XBot::Logger::error("Unable to set velocity reference for task '%s' (frame_id '%s' undefined)\n", 
                                ee_name.c_str(), 
                                msg->header.frame_id.c_str()
            );
            
            return;
        }
        
    }
    
    vel.head<3>() = b_R_f * vel.head<3>();
    vel.tail<3>() = b_R_f * vel.tail<3>();
    _cartesian_interface->setVelocityReference(ee_name, vel);
    
}

void RosServerClass::init_online_pos_topics()
{
    for(std::string ee_name : _cartesian_interface->getTaskList())
    {
        std::string topic_name = "" + ee_name + "/reference";

        auto cb = std::bind(&RosServerClass::online_position_reference_cb, this, std::placeholders::_1, ee_name);

        ros::Subscriber sub = _nh.subscribe<geometry_msgs::PoseStamped>(topic_name,
                                                                        1, cb);

        _pos_sub.push_back(sub);
    }
}

void RosServerClass::init_reach_pose_action_servers()
{
    for(std::string ee_name : _cartesian_interface->getTaskList())
    {
        _action_managers.emplace_back(_nh, ee_name, _cartesian_interface);
    }
}

void RosServerClass::manage_reach_actions()
{
    for(auto& action_manager : _action_managers)
    {
        action_manager.run();
    }
}

void RosServerClass::publish_state(ros::Time time)
{
    int i = 0;
    for(std::string ee_name : _cartesian_interface->getTaskList())
    {
        geometry_msgs::PoseStamped msg;
        std::string frame_id = _cartesian_interface->getBaseLink(ee_name);
        frame_id = frame_id == "world" ? "world_odom" : frame_id;
        msg.header.stamp = time;
        msg.header.frame_id = _tf_prefix_slash + frame_id;
        Eigen::Affine3d base_T_ee;
        _cartesian_interface->getPoseReference(ee_name, base_T_ee);
        tf::poseEigenToMsg(base_T_ee, msg.pose);
        _state_pub[i].publish(msg);
        i++;
    }
}

void XBot::Cartesian::RosServerClass::publish_posture_state(ros::Time time)
{
    sensor_msgs::JointState msg;
    Eigen::VectorXd _posture_ref;
    
    if(_posture_pub.getNumSubscribers() == 0)
    {
        return;
    }
    
    if(!_cartesian_interface->getReferencePosture(_posture_ref))
    {
        return;
    }
    
    msg.header.stamp = time;
    msg.name.reserve(_model->getJointNum());
    msg.position.reserve(_model->getJointNum());
    int i = 0;
    for(const std::string& jname : _model->getEnabledJointNames())
    {
        msg.name.push_back(jname);
        msg.position.push_back(_posture_ref[i]);
        i++;
    }
    
    _posture_pub.publish(msg);
}   

void XBot::Cartesian::RosServerClass::publish_solution(ros::Time time)
{
    sensor_msgs::JointState msg;
    Eigen::VectorXd _sol_q, _sol_qdot, _sol_tau;

    if(_solution_pub.getNumSubscribers() == 0)
    {
        return;
    }
    
    _model->getJointPosition(_sol_q);
    _model->getJointVelocity(_sol_qdot);
    _model->getJointEffort(_sol_tau);
    
    msg.header.stamp = time;
    msg.name.reserve(_model->getJointNum());
    msg.position.reserve(_model->getJointNum());
    msg.velocity.reserve(_model->getJointNum());
    int i = 0;
    for(const std::string& jname : _model->getEnabledJointNames())
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
    
    manage_reach_actions();

    ros::Time now = ros::Time::now();
    publish_state(now);
    publish_ref_tf(now);
    publish_world_tf(now);
    publish_posture_state(now);
    publish_solution(now);
    
    if(_ros_enabled_ci)
    {
        _ros_enabled_ci->updateRos();
    }

}

RosServerClass::RosServerClass(CartesianInterface::Ptr intfc, 
                               ModelInterface::ConstPtr model, 
                               Options opt):
    _cartesian_interface(intfc),
    _model(model),
    _opt(opt),
    _nh(opt.ros_namespace)
{
    _tf_prefix = _opt.tf_prefix;
    _tf_prefix_slash = _tf_prefix == "" ? "" : (_tf_prefix + "/");
    
    _nh.setCallbackQueue(&_cbk_queue);
    
    init_online_pos_topics();
    init_online_vel_topics();
    init_interaction_topics();
    init_reach_pose_action_servers();
    init_state_broadcasting();
    init_rspub();
    init_task_info_services();
    init_interaction_srvs();
    init_task_info_setters();
    init_task_list_service();
    init_postural_task_topics_and_services();
    init_update_param_services();
    init_reset_world_service();

    _ros_enabled_ci = std::dynamic_pointer_cast<RosEnabled>(_cartesian_interface);
    if(!_ros_enabled_ci || !_ros_enabled_ci->initRos(_nh))
    {
        _ros_enabled_ci.reset();
    }
    else
    {
        XBot::Logger::success("Ros interface initialized correctly \n");
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
                                                       _tf_prefix_slash + "world_odom"));
    

    /* Publish ref-to-actual-robot fixed tf */
    if(_tf_prefix != "")
    {
        Eigen::Affine3d T_eye;
        T_eye.setIdentity();
        tf::Transform transform_eye;
        tf::transformEigenToTF(T_eye, transform_eye);

        _tf_broadcaster.sendTransform(tf::StampedTransform(transform_eye,
                                                        time,
                                                        _tf_prefix_slash + "world",
                                                        "world"));
    }
}


void XBot::Cartesian::RosServerClass::init_task_list_service()
{
    std::string srv_name = "get_task_list";
    _tasklist_srv = _nh.advertiseService(srv_name, &RosServerClass::task_list_cb, this);
}

bool XBot::Cartesian::RosServerClass::task_list_cb(cartesian_interface::GetTaskListRequest& req,
                                                   cartesian_interface::GetTaskListResponse& res)
{
    
    for(const auto& task : _cartesian_interface->getTaskList())
    {
        res.distal_links.push_back(task);
        res.base_links.push_back(_cartesian_interface->getBaseLink(task));
    }
    
    return true;
}



void XBot::Cartesian::RosServerClass::init_task_info_services()
{
    for(std::string ee_name : _cartesian_interface->getTaskList())
    {
        std::string srv_name = "" + ee_name + "/get_task_properties";
        auto cb = boost::bind(&RosServerClass::get_task_info_cb,
                            this,
                            _1,
                            _2,
                            ee_name);
        ros::ServiceServer srv = _nh.advertiseService<cartesian_interface::GetTaskInfoRequest,
                                                      cartesian_interface::GetTaskInfoResponse>(srv_name, cb);

        _get_task_info_srv.push_back(srv);
    }
}

bool XBot::Cartesian::RosServerClass::reset_params_cb(std_srvs::TriggerRequest& req, std_srvs::TriggerResponse& res)
{
    for(std::string task : _cartesian_interface->getTaskList())
    {
        
        double max_vel_lin, max_vel_ang, max_acc_lin, max_acc_ang;
        _cartesian_interface->getVelocityLimits(task, max_vel_lin, max_vel_ang);
        _cartesian_interface->getAccelerationLimits(task, max_acc_lin, max_acc_ang);

        _nh.getParam(task + "/max_velocity_linear", max_vel_lin);
        _nh.getParam(task + "/max_velocity_angular", max_vel_ang);
        _nh.getParam(task + "/max_acceleration_linear", max_acc_lin);
        _nh.getParam(task + "/max_acceleration_angular", max_acc_ang);
        
        _cartesian_interface->setVelocityLimits(task, max_vel_lin, max_vel_ang);
        _cartesian_interface->setAccelerationLimits(task, max_acc_lin, max_acc_ang);
    }
    
    res.message = "Successfully updated velocity limits for all tasks";
    res.success = true;
    
    return true;
}

void XBot::Cartesian::RosServerClass::init_update_param_services()
{
    
    for(std::string task : _cartesian_interface->getTaskList())
    {
        
        double max_vel_lin, max_vel_ang, max_acc_lin, max_acc_ang;
        _cartesian_interface->getVelocityLimits(task, max_vel_lin, max_vel_ang);
        _cartesian_interface->getAccelerationLimits(task, max_acc_lin, max_acc_ang);
        
        if(!_nh.hasParam(task + "/max_velocity_linear"))
            _nh.setParam(task + "/max_velocity_linear", max_vel_lin);
        if(!_nh.hasParam(task + "/max_velocity_angular"))
            _nh.setParam(task + "/max_velocity_angular", max_vel_ang);
        if(!_nh.hasParam(task + "/max_acceleration_linear"))
            _nh.setParam(task + "/max_acceleration_linear", max_acc_lin);
        if(!_nh.hasParam(task + "/max_acceleration_angular"))
            _nh.setParam(task + "/max_acceleration_angular", max_acc_ang);
        
    }
    
    std_srvs::TriggerRequest req;
    std_srvs::TriggerResponse res;
    reset_params_cb(req, res);
    
    std::string srv_name = "update_velocity_limits";
    auto cb = boost::bind(&RosServerClass::reset_params_cb,
                        this,
                        _1,
                        _2);
    
    _update_limits_srv = _nh.advertiseService<std_srvs::TriggerRequest,
                                              std_srvs::TriggerResponse>(srv_name, cb);
}




bool XBot::Cartesian::RosServerClass::get_task_info_cb(cartesian_interface::GetTaskInfoRequest& req,
                                                       cartesian_interface::GetTaskInfoResponse& res,
                                                       const std::string& ee_name)
{

    res.base_link = _cartesian_interface->getBaseLink(ee_name);
    res.control_mode  =  CartesianInterface::ControlTypeAsString(_cartesian_interface->getControlMode(ee_name));
    res.task_state  =  CartesianInterface::StateAsString(_cartesian_interface->getTaskState(ee_name));
    res.distal_link = ee_name;
    res.task_interface = CartesianInterface::TaskInterfaceAsString(_cartesian_interface->getTaskInterface(ee_name));
    
    return true;
}




RosServerClass::~RosServerClass()
{
}

XBot::ModelInterface::ConstPtr RosServerClass::getModel() const
{
    return _model;
}

void XBot::Cartesian::RosServerClass::init_task_info_setters()
{
    for(std::string ee_name : _cartesian_interface->getTaskList())
    {
        std::string srv_name = "" + ee_name + "/set_task_properties";
        auto cb = boost::bind(&RosServerClass::set_task_info_cb,
                            this,
                            _1,
                            _2,
                            ee_name);
        ros::ServiceServer srv = _nh.advertiseService<cartesian_interface::SetTaskInfoRequest,
                                                      cartesian_interface::SetTaskInfoResponse>(srv_name, cb);

        _set_task_info_srv.push_back(srv);
    }
}

bool XBot::Cartesian::RosServerClass::set_task_info_cb(cartesian_interface::SetTaskInfoRequest& req,
                                                       cartesian_interface::SetTaskInfoResponse& res, 
                                                       const std::string& ee_name)
{
    std::string new_base_link = req.base_link;
    if(new_base_link == "world_odom")
    {
        new_base_link = "world";
    }
    
    if(new_base_link != "" && _cartesian_interface->setBaseLink(ee_name, new_base_link))
    {
        res.message = "Successfully set base link of task " + ee_name + " to " + new_base_link;
        res.success = true;
    }
    else if(new_base_link != "")
    {
        res.message = "Unable to set base link of task " + ee_name + " to " + new_base_link;
        res.success = false;
    }
    
    
    if(req.control_mode != "" && _cartesian_interface->setControlMode(ee_name, CartesianInterface::ControlTypeFromString(req.control_mode)))
    {
        res.message = "Successfully set control mode of task " + ee_name + " to " + req.control_mode;
        res.success = true;
    }
    else if(req.control_mode != "")
    {
        res.message = "Unable to set control mode of task " + ee_name + " to " + req.control_mode;
        res.success = false;
    }
    
    
    return true;
}


void XBot::Cartesian::RosServerClass::init_postural_task_topics_and_services()
{
    std::string postural_ref_topic_name = "posture/reference";
    std::string postural_state_topic_name = "posture/state";
    std::string postural_srv_name = "posture/reset";
    
    _posture_sub = _nh.subscribe(postural_ref_topic_name, 1, &RosServerClass::online_posture_reference_cb, this);
    _posture_pub = _nh.advertise<sensor_msgs::JointState>(postural_state_topic_name, 1);
    _reset_posture_srv = _nh.advertiseService(postural_srv_name, &RosServerClass::reset_posture_cb, this);
    
    std_srvs::TriggerRequest req;
    std_srvs::TriggerResponse res;
    reset_posture_cb(req, res);
}

void XBot::Cartesian::RosServerClass::online_posture_reference_cb(const sensor_msgs::JointStateConstPtr& msg)
{
    JointNameMap posture_ref;
    
    for(int i = 0; i < msg->name.size(); i++)
    {
        if(msg->position.size() <= i)
        {
            continue;
        }
        
        posture_ref[msg->name[i]] = msg->position[i];
    }
    
    _cartesian_interface->setReferencePosture(posture_ref);
}

bool XBot::Cartesian::RosServerClass::reset_posture_cb(std_srvs::TriggerRequest& req, 
                                                       std_srvs::TriggerResponse& res)
{
    JointNameMap posture_ref;
    _model->getJointPosition(posture_ref);
    
    if(_cartesian_interface->setReferencePosture(posture_ref))
    {
        res.message = "Reference posture reset successfully";
        res.success = true;
    }
    else
    {
        res.message = "Unable to reset reference posture (no postural task defined?)";
        res.success = false;
    }
    
    std::map<std::string, double> zeros_map(posture_ref.begin(), posture_ref.end());
    _nh.setParam("posture/home", zeros_map);
    
    return true;
}

void XBot::Cartesian::RosServerClass::init_reset_world_service()
{
    _reset_world_srv = _nh.advertiseService("reset_world", &RosServerClass::reset_world_cb, this);
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
    
    if(_cartesian_interface->resetWorld(new_world))
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


bool RosServerClass::get_impedance_cb(cartesian_interface::GetImpedanceRequest& req, 
                                      cartesian_interface::GetImpedanceResponse& res, 
                                      const std::string& ee_name)
{
    Eigen::Vector6d f;
    Eigen::Matrix6d k, d;
    
    if(!_cartesian_interface->getDesiredInteraction(ee_name, f, k, d))
    {
        return false;
    }
    
    for(int i = 0; i < 3; i++)
    {
        for(int j = 0; j < 3; j++)
        {
            res.linear.stiffness[i+3*j] = k(i,j);
            res.linear.damping[i+3*j] = d(i,j);
            
            res.angular.stiffness[i+3*j] =  k(i+3,j+3);
            res.angular.damping[i+3*j] = d(i+3,j+3);
        }
    }
    
    
    return true;
}

void RosServerClass::online_impedance_reference_cb(const cartesian_interface::Impedance6ConstPtr& msg, 
                                                     const std::string& ee_name)
{
    Eigen::Matrix6d k, d;
    k.setZero();
    d.setZero();
    for(int i = 0; i < 3; i++)
    {
        for(int j = i; j < 3; j++)
        {
            k(i,j) = k(j,i) = msg->linear.stiffness[i+3*j];
            d(i,j) = d(j,i) = msg->linear.damping[i+3*j];
            
            k(i+3,j+3) = k(j+3,i+3) = msg->angular.stiffness[i+3*j];
            d(i+3,j+3) = d(j+3,i+3) = msg->angular.damping[i+3*j];
        }
    }
    
    if((k.diagonal().array() >= 0).all() && (d.diagonal().array() >= 0).all())
    {
        _cartesian_interface->setDesiredStiffness(ee_name, k);
        _cartesian_interface->setDesiredDamping(ee_name, d);
    }
}




ReachActionManager::ReachActionManager(ros::NodeHandle nh,
                                       std::string ee_name,
                                       CartesianInterface::Ptr ci):
    _action_server(new ActionServer(nh, ee_name + "/reach", false)),
    _cartesian_interface(ci),
    _state(ReachActionState::IDLE),
    _ee_name(ee_name)
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
                     "Accepted new goal for task '%s'\n", _ee_name.c_str());

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
         _cartesian_interface->getCurrentPose(_ee_name, base_T_ee);

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
        if(!_cartesian_interface->setWayPoints(_ee_name, waypoints))
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
    if(_cartesian_interface->getTaskState(_ee_name) == State::Reaching)
    {
        Logger::info(Logger::Severity::HIGH,
                     "Reaching started for task '%s'\n", _ee_name.c_str());

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
                           _ee_name.c_str());

        _cartesian_interface->abort(_ee_name);

        cartesian_interface::ReachPoseResult result;
        Eigen::Affine3d base_T_ee, base_T_ref, ref_T_ee;
        _cartesian_interface->getCurrentPose(_ee_name, base_T_ee);
        _cartesian_interface->getPoseReference(_ee_name, base_T_ref);
        ref_T_ee = base_T_ref.inverse() * base_T_ee;

        tf::poseEigenToMsg(base_T_ref, result.final_frame);
        result.position_error_norm = ref_T_ee.translation().norm();
        result.orientation_error_angle = Eigen::AngleAxisd(ref_T_ee.linear()).angle();

        _action_server->setPreempted(result);
        _state = ReachActionState::COMPLETED; // next state is 'completed'
        return;
    }

    // trajectory ended
    if(_cartesian_interface->getTaskState(_ee_name) == State::Online)
    {

        cartesian_interface::ReachPoseResult result;
        Eigen::Affine3d base_T_ee, base_T_ref, ref_T_ee;
        _cartesian_interface->getCurrentPose(_ee_name, base_T_ee);
        _cartesian_interface->getPoseReference(_ee_name, base_T_ref);
        ref_T_ee = base_T_ref.inverse() * base_T_ee;

        tf::poseEigenToMsg(base_T_ee, result.final_frame);
        result.position_error_norm = ref_T_ee.translation().norm();
        result.orientation_error_angle = Eigen::AngleAxisd(ref_T_ee.linear()).angle();

        XBot::Logger::success(XBot::Logger::Severity::HIGH,
                              "Goal for task '%s' succeded \n"
                              "Final position error norm: %f m \n"
                              "Final orientation error: %f rad \n",
                              _ee_name.c_str(),
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
        feedback.current_reference.header.frame_id = _cartesian_interface->getBaseLink(_ee_name);
        _cartesian_interface->getPoseReference(_ee_name, base_T_ref);
        tf::poseEigenToMsg(base_T_ref, feedback.current_reference.pose);

        Eigen::Affine3d base_T_ee;
        feedback.current_pose.header.stamp = ros::Time::now();
        feedback.current_pose.header.frame_id = _cartesian_interface->getBaseLink(_ee_name);
        _cartesian_interface->getCurrentPose(_ee_name, base_T_ee);
        tf::poseEigenToMsg(base_T_ee, feedback.current_pose.pose);

        feedback.current_segment_id = _cartesian_interface->getCurrentSegmentId(_ee_name);

        _action_server->publishFeedback(feedback);
        return; // next state is 'running'

    }
}

void ReachActionManager::run_state_completed()
{
    XBot::Logger::info(XBot::Logger::Severity::HIGH,
                               "Goal for task '%s' completed\n",
                               _ee_name.c_str());

    _state = ReachActionState::IDLE;
    return;
}
