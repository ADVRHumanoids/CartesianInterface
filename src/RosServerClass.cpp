#include <cartesian_interface/ros/RosServerClass.h>

using namespace XBot::Cartesian;

RosServerClass::Options::Options():
    spawn_markers(true)
{

}

void RosServerClass::__generate_state_broadcasting()
{
    for(std::string ee_name : _cartesian_interface->getTaskList())
    {
        std::string topic_name = "/xbotcore/cartesian/" + ee_name + "/state";

        ros::Publisher pub = _nh.advertise<geometry_msgs::PoseStamped>(topic_name,
                                                                        1);

        _state_pub.push_back(pub);
    }
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
}

void RosServerClass::publish_ref_tf()
{
    /* Publish TF */
    XBot::JointNameMap _joint_name_map;
    _model->getJointPosition(_joint_name_map);
    std::map<std::string, double> _joint_name_std_map(_joint_name_map.begin(), _joint_name_map.end());

    _rspub->publishTransforms(_joint_name_std_map, ros::Time::now(), "ci");
    _rspub->publishFixedTransforms("ci");

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




void RosServerClass::__generate_online_pos_topics()
{
    for(std::string ee_name : _cartesian_interface->getTaskList())
    {
        std::string topic_name = "/xbotcore/cartesian/" + ee_name + "/reference";

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
        std::string action_name = "/xbotcore/cartesian/" + ee_name + "/reach";

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

void RosServerClass::publish_state()
{
    int i = 0;
    for(std::string ee_name : _cartesian_interface->getTaskList())
    {
        geometry_msgs::PoseStamped msg;
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = _cartesian_interface->getBaseLink(ee_name);
        Eigen::Affine3d base_T_ee;
        _cartesian_interface->getCurrentPose(ee_name, base_T_ee);
        tf::poseEigenToMsg(base_T_ee, msg.pose);
        _state_pub[i].publish(msg);
        i++;
    }
}


void RosServerClass::run()
{
    ros::spinOnce();

    manage_reach_actions();

    publish_state();

    publish_ref_tf();
    publish_world_tf();

}

RosServerClass::RosServerClass(CartesianInterface::Ptr intfc, 
                               ModelInterface::ConstPtr model, 
                               Options opt):
    _cartesian_interface(intfc),
    _model(model),
    _opt(opt)
{
    __generate_online_pos_topics();
    __generate_reach_pose_action_servers();
    __generate_state_broadcasting();
    __generate_toggle_pos_mode_services();
    __generate_toggle_task_services();
    __generate_rspub();
    __generate_task_info_services();
    __generate_task_info_setters();

    if(_opt.spawn_markers)
    {
        _marker_thread = std::make_shared<std::thread>(&RosServerClass::__generate_markers, this);
    }
}

void RosServerClass::publish_world_tf()
{
    /* Publish world odom */
    Eigen::Affine3d w_T_pelvis;
    _model->getFloatingBasePose(w_T_pelvis);
    tf::Transform transform;
    tf::transformEigenToTF(w_T_pelvis, transform);
    std::string fb_link;
    _model->getFloatingBaseLink(fb_link);

    _tf_broadcaster.sendTransform(tf::StampedTransform(transform.inverse(),
                                                       ros::Time::now(),
                                                       "ci/"+fb_link,
                                                       "ci/world_odom"));

    /* Publish ref-to-actual-robot fixed tf */
    Eigen::Affine3d T_eye;
    T_eye.setIdentity();
    tf::Transform transform_eye;
    tf::transformEigenToTF(T_eye, transform_eye);

    _tf_broadcaster.sendTransform(tf::StampedTransform(transform_eye,
                                                       ros::Time::now(),
                                                       "ci/world",
                                                       "world"));
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

void RosServerClass::__generate_toggle_pos_mode_services()
{
    for(std::string ee_name : _cartesian_interface->getTaskList())
    {
        std::string srv_name = "/xbotcore/cartesian/" + ee_name + "/toggle_position_mode";
        auto cb = boost::bind(&RosServerClass::toggle_position_mode_cb,
                            this,
                            _1,
                            _2,
                            ee_name);
        ros::ServiceServer srv = _nh.advertiseService<std_srvs::SetBoolRequest, std_srvs::SetBoolResponse>(srv_name, cb);

        _toggle_pos_mode_srv.push_back(srv);
    }
}

bool RosServerClass::toggle_position_mode_cb(std_srvs::SetBoolRequest& req,
                                             std_srvs::SetBoolResponse& res,
                                             const std::string& ee_name)
{
    if(req.data)
    {
        _cartesian_interface->setControlMode(ee_name, CartesianInterface::ControlType::Position);
        res.message = "Control mode set to position for end effector " + ee_name;
    }
    else
    {
        _cartesian_interface->setControlMode(ee_name, CartesianInterface::ControlType::Velocity);
        res.message = "Control mode set to velocity for end effector " + ee_name;
    }

    res.success = true;

    return true;
}

void RosServerClass::__generate_toggle_task_services()
{
    for(std::string ee_name : _cartesian_interface->getTaskList())
    {
        std::string srv_name = "/xbotcore/cartesian/" + ee_name + "/activate_task";
        auto cb = boost::bind(&RosServerClass::toggle_task_cb,
                            this,
                            _1,
                            _2,
                            ee_name);
        ros::ServiceServer srv = _nh.advertiseService<std_srvs::SetBoolRequest, std_srvs::SetBoolResponse>(srv_name, cb);

        _toggle_task_srv.push_back(srv);
    }
}

void XBot::Cartesian::RosServerClass::__generate_task_info_services()
{
    for(std::string ee_name : _cartesian_interface->getTaskList())
    {
        std::string srv_name = "/xbotcore/cartesian/" + ee_name + "/get_task_properties";
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

bool RosServerClass::toggle_task_cb(std_srvs::SetBoolRequest& req,
                                    std_srvs::SetBoolResponse& res,
                                    const std::string& ee_name)
{
    if(req.data)
    {
        _cartesian_interface->setControlMode(ee_name, CartesianInterface::ControlType::Position);
        res.message = "Control mode set to position for end effector " + ee_name;
    }
    else
    {
        _cartesian_interface->setControlMode(ee_name, CartesianInterface::ControlType::Disabled);
        res.message = "Control mode set to disabled for end effector " + ee_name;
    }

    res.success = true;
    return true;
}

void RosServerClass::__generate_markers()
{

    for(std::string ee_name : _cartesian_interface->getTaskList())
    {
        if(ee_name == "com")
        {
            continue;
        }
        
        std::string base_link = _cartesian_interface->getBaseLink(ee_name);
        base_link = base_link == "world" ? "world_odom" : base_link;
        auto marker = std::make_shared<CartesianMarker>(base_link,
                                                   ee_name,
                                                   static_cast<const urdf::Model&>(_model->getUrdf()),
                                                   "ci/"
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
    _marker_thread->join();
}

XBot::ModelInterface::ConstPtr RosServerClass::getModel() const
{
    return _model;
}

void XBot::Cartesian::RosServerClass::__generate_task_info_setters()
{
    for(std::string ee_name : _cartesian_interface->getTaskList())
    {
        std::string srv_name = "/xbotcore/cartesian/" + ee_name + "/set_task_properties";
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
    if(_cartesian_interface->setBaseLink(ee_name, req.base_link))
    {
        res.message = "Successfully set base link of task " + ee_name + " to " + req.base_link;
        res.success = true;
        _markers.at(ee_name)->setBaseLink(req.base_link);
    }
    else
    {
        res.message = "Unable to set base link of task " + ee_name + " to " + req.base_link;
        res.success = false;
    }
    return true;
}



