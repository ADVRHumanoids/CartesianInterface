#include <cartesian_interface/ros/RosServerClass.h>
#include <cartesian_interface/GetTaskListResponse.h>

using namespace XBot::Cartesian;

RosServerClass::Options::Options():
    spawn_markers(true),
    tf_prefix("ci")
{

}

void RosServerClass::__generate_state_broadcasting()
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


void RosServerClass::__generate_rspub()
{
    KDL::Tree kdl_tree;
    kdl_parser::treeFromUrdfModel(_model->getUrdf(), kdl_tree);

    _rspub.reset(new RsPub(kdl_tree));

    std::string _urdf_param_name = "/xbotcore/" + _model->getUrdf().getName() + "/robot_description";
    std::string _tf_prefix = "/xbotcore/" + _model->getUrdf().getName();
    _nh.setParam(_urdf_param_name, _model->getUrdfString());
    _nh.setParam("/robot_description", _model->getUrdfString());
    _nh.setParam("/robot_description_semantic", _model->getSrdfString());
    
    _com_pub = _nh.advertise<geometry_msgs::PointStamped>("com_position", 1);
}

void RosServerClass::publish_ref_tf(ros::Time time)
{
    /* Publish TF */
    XBot::JointNameMap _joint_name_map;
    _model->getJointPosition(_joint_name_map);
    std::map<std::string, double> _joint_name_std_map(_joint_name_map.begin(), _joint_name_map.end());

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
    
    if(ee_name == "com")
    {
        _cartesian_interface->setComPositionReference(T.translation());
    }
    else
    {
        _cartesian_interface->setPoseReference(ee_name, T);
    }
}



void RosServerClass::__generate_online_vel_topics()
{
    for(std::string ee_name : _cartesian_interface->getTaskList())
    {
        std::string topic_name = "" + ee_name + "/velocity_reference";

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
        XBot::Logger::error("frame_id = world not supported (TBD)\n");
        return;
    }
    
    if(msg->header.frame_id != "")
    {
        Eigen::Matrix3d b_R_f;
        b_R_f.setIdentity();
        
        if(!_model->getOrientation(msg->header.frame_id, _cartesian_interface->getBaseLink(ee_name), b_R_f))
        {
            XBot::Logger::error("Unable to set velocity reference for task %s (frame_id \"%s\" undefined)\n", 
                                    ee_name.c_str(), 
                                    msg->header.frame_id.c_str()
            );
            
            return;
        }
        
        vel.head<3>() = b_R_f * vel.head<3>();
        vel.tail<3>() = b_R_f * vel.tail<3>();
    }
    
    
    if(ee_name == "com")
    {
        Eigen::Vector3d com_ref, com_vref;
        _cartesian_interface->getComPositionReference(com_ref);
        com_vref = vel.head<3>();
        
        _cartesian_interface->setComPositionReference(com_ref, com_vref);
    }
    else
    {
        Eigen::Affine3d T;
        _cartesian_interface->getPoseReference(ee_name, T);
        _cartesian_interface->setPoseReference(ee_name, T, vel);
    }
}

void RosServerClass::__generate_online_pos_topics()
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

void RosServerClass::__generate_reach_pose_action_servers()
{
    for(std::string ee_name : _cartesian_interface->getTaskList())
    {
        std::string action_name = "" + ee_name + "/reach";

        _action_servers.emplace_back( new ActionServer(_nh, action_name, false) );

        _action_servers.back()->start();

        _is_action_active.push_back(false);
    }
}

void RosServerClass::manage_reach_actions()
{
    /* Poll */
    for(int i = 0; i < _action_servers.size(); i++)
    {
        ActionServerPtr as = _action_servers[i];

        const std::string& ee_name = _cartesian_interface->getTaskList()[i];

        CartesianInterface::State current_state = _cartesian_interface->getTaskState(ee_name);

        if(as->isNewGoalAvailable() && current_state != CartesianInterface::State::Reaching)
        {
            if(!_is_action_active[i])
            {
                auto goal = as->acceptNewGoal();
                
                if(goal->frames.size() != goal->time.size())
                {
                    as->setAborted(cartesian_interface::ReachPoseResult(), "Time and frame size must match");
                    continue;
                }
                
                Eigen::Affine3d base_T_ee;
                 _cartesian_interface->getCurrentPose(ee_name, base_T_ee);
                 
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
                
                Eigen::Affine3d T_first_ref = waypoints[0].frame;

                
                
                if(ee_name == "com"){
                    
                    if(!_cartesian_interface->setTargetComPosition(T_first_ref.translation(), goal->time.back()))
                    {
                        as->setAborted(cartesian_interface::ReachPoseResult(), "Internal error");
                        continue;
                    }
                    
                }
                else
                {
                    
                    if(!_cartesian_interface->setWayPoints(ee_name, waypoints))
                    {
                        as->setAborted(cartesian_interface::ReachPoseResult(), "Internal error");
                        continue;
                    }
                }

                current_state = _cartesian_interface->getTaskState(ee_name);

            }
        }

        if(as->isActive() && as->isPreemptRequested())
        {
            XBot::Logger::info(XBot::Logger::Severity::HIGH, "Goal for task %s canceled by user\n", ee_name.c_str());
            _cartesian_interface->abort(ee_name);

            cartesian_interface::ReachPoseResult result;
            Eigen::Affine3d base_T_ee, base_T_ref, ref_T_ee;
            _cartesian_interface->getCurrentPose(ee_name, base_T_ee);
            _cartesian_interface->getPoseReference(ee_name, base_T_ref);
            ref_T_ee = base_T_ref.inverse() * base_T_ee;

            tf::poseEigenToMsg(base_T_ref, result.final_frame);
            result.position_error_norm = ref_T_ee.translation().norm();
            result.orientation_error_angle = Eigen::AngleAxisd(ref_T_ee.linear()).angle();

            as->setPreempted(result);
        }

        if(as->isActive())
        {
            if(current_state == CartesianInterface::State::Online)
            {
                
                cartesian_interface::ReachPoseResult result;
                Eigen::Affine3d base_T_ee, base_T_ref, ref_T_ee;
                _cartesian_interface->getCurrentPose(ee_name, base_T_ee);
                _cartesian_interface->getPoseReference(ee_name, base_T_ref);
                ref_T_ee = base_T_ref.inverse() * base_T_ee;

                tf::poseEigenToMsg(base_T_ee, result.final_frame);
                result.position_error_norm = ref_T_ee.translation().norm();
                result.orientation_error_angle = Eigen::AngleAxisd(ref_T_ee.linear()).angle();
                
                XBot::Logger::success(XBot::Logger::Severity::HIGH, "Goal for task %s succeded\nFinal position error norm: %f m \nFinal orientation error: %f rad\n", 
                    ee_name.c_str(), result.position_error_norm, result.orientation_error_angle);

                as->setSucceeded(result);
            }
            else
            {
                cartesian_interface::ReachPoseFeedback feedback;
                feedback.current_reference.header.stamp = ros::Time::now();
                feedback.current_reference.header.frame_id = _cartesian_interface->getBaseLink(ee_name);
                Eigen::Affine3d base_T_ref;
                _cartesian_interface->getPoseReference(ee_name, base_T_ref);

                tf::poseEigenToMsg(base_T_ref, feedback.current_reference.pose);
                as->publishFeedback(feedback);

            }
        }
    }
}

void RosServerClass::publish_state(ros::Time time)
{
    int i = 0;
    for(std::string ee_name : _cartesian_interface->getTaskList())
    {
        geometry_msgs::PoseStamped msg;
        msg.header.stamp = time;
        msg.header.frame_id = _cartesian_interface->getBaseLink(ee_name);
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
    
    if(!_cartesian_interface->getReferencePosture(_posture_ref))
    {
        return;
    }
    
    if(_posture_pub.getNumSubscribers() == 0)
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
    Eigen::VectorXd _sol_q, _sol_qdot;

    if(_solution_pub.getNumSubscribers() == 0)
    {
        return;
    }
    
    _model->getJointPosition(_sol_q);
    _model->getJointVelocity(_sol_qdot);
    
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

}

RosServerClass::RosServerClass(CartesianInterface::Ptr intfc, 
                               ModelInterface::ConstPtr model, 
                               Options opt):
    _cartesian_interface(intfc),
    _model(model),
    _opt(opt),
    _nh("cartesian")
{
    _tf_prefix = _opt.tf_prefix;
    _tf_prefix_slash = _tf_prefix == "" ? "" : (_tf_prefix + "/");
    
    _nh.setCallbackQueue(&_cbk_queue);
    
    __generate_online_pos_topics();
    __generate_online_vel_topics();
    __generate_reach_pose_action_servers();
    __generate_state_broadcasting();
    __generate_rspub();
    __generate_task_info_services();
    __generate_task_info_setters();
    __generate_task_list_service();
    __generate_postural_task_topics_and_services();
    __generate_update_param_services();
    __generate_reset_world_service();

    if(_opt.spawn_markers)
    {
        _marker_thread = std::make_shared<std::thread>(&RosServerClass::__generate_markers, this);
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


geometry_msgs::Pose RosServerClass::get_normalized_pose(const geometry_msgs::Pose& pose)
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

void XBot::Cartesian::RosServerClass::__generate_task_list_service()
{
    std::string srv_name = "get_task_list";
    _tasklist_srv = _nh.advertiseService(srv_name, &RosServerClass::task_list_cb, this);
}

bool XBot::Cartesian::RosServerClass::task_list_cb(cartesian_interface::GetTaskListRequest& req,
                                                   cartesian_interface::GetTaskListResponse& res)
{
    
    for(const auto& task : _cartesian_interface->getTaskList())
    {
        std_msgs::String dist_str; dist_str.data = task;
        std_msgs::String base_str; base_str.data = _cartesian_interface->getBaseLink(task);
        res.distal_links.push_back(dist_str);
        res.base_links.push_back(base_str);
    }
    
    return true;
}



void XBot::Cartesian::RosServerClass::__generate_task_info_services()
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

void XBot::Cartesian::RosServerClass::__generate_update_param_services()
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



void RosServerClass::__generate_markers()
{

    for(std::string ee_name : _cartesian_interface->getTaskList())
    {
        unsigned int control_type;
        
        if(ee_name == "com")
        {
            control_type = visualization_msgs::InteractiveMarkerControl::MOVE_3D;
        }
        else
        {
            control_type = visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE_3D;
        }
        
        std::string base_link = _cartesian_interface->getBaseLink(ee_name);
        base_link = base_link == "world" ? "world_odom" : base_link;
        auto marker = std::make_shared<CartesianMarker>(base_link,
                                                   ee_name,
                                                   static_cast<const urdf::Model&>(_model->getUrdf()),
                                                   control_type,
                                                   _tf_prefix_slash
                                                  );
        
        _markers[ee_name] = marker;
    }
}

bool XBot::Cartesian::RosServerClass::get_task_info_cb(cartesian_interface::GetTaskInfoRequest& req,
                                                       cartesian_interface::GetTaskInfoResponse& res,
                                                       const std::string& ee_name)
{

    res.base_link = _cartesian_interface->getBaseLink(ee_name);
    res.control_mode  =  CartesianInterface::ControlTypeAsString(_cartesian_interface->getControlMode(ee_name));
    res.task_state  =  CartesianInterface::StateAsString(_cartesian_interface->getTaskState(ee_name));
    res.distal_link = ee_name;
    
    return true;
}




RosServerClass::~RosServerClass()
{
    if(_marker_thread) _marker_thread->join();
}

XBot::ModelInterface::ConstPtr RosServerClass::getModel() const
{
    return _model;
}

void XBot::Cartesian::RosServerClass::__generate_task_info_setters()
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
        
        std::string __new_base_link = new_base_link == "world" ? "world_odom" : new_base_link;
        if(_markers.count(ee_name) != 0)
        {
            _markers.at(ee_name)->setBaseLink(__new_base_link);
        }
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


void XBot::Cartesian::RosServerClass::__generate_postural_task_topics_and_services()
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

void XBot::Cartesian::RosServerClass::__generate_reset_world_service()
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



