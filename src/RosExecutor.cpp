#include <cartesian_interface/ros/RosExecutor.h>
#include <xbot_msgs/JointState.h>

using namespace XBot::Cartesian;

RosExecutor::RosExecutor(std::string ns):
    _nh(ns),
    _nh_priv("~"),
    _logger(MatLogger::getLogger("/tmp/ros_executor_log"))
{
    
    init_ros();
    
    init_load_config();
    
    init_load_robot();
    
    init_customize_command();
    
    init_load_model();
    
    init_load_world_frame();
    
    init_load_torque_offset();
    
    init_create_loop_timer();
    
    
}

void RosExecutor::init_ros()
{
    /* Broadcast a notification whenever a new controller is loaded */
    _ctrl_changed_pub = _nh.advertise<std_msgs::Empty>("changed_controller_event", 1);
    
    /* Load controller service */
    _loader_srv = _nh.advertiseService("load_controller", &RosExecutor::loader_callback, this);
    
    /* Load reset service */
    _reset_srv = _nh.advertiseService("reset", &RosExecutor::reset_callback, this);
}

void RosExecutor::init_load_config()
{
    /* Obtain xbot config object */
    bool use_xbot_config = _nh_priv.param("use_xbot_config", false);
    
    if(use_xbot_config)
    {
        _options_source = Utils::LoadFrom::CONFIG;
        Logger::info("Configuring from file %s\n", XBot::Utils::getXBotConfig().c_str());
    }
    else
    {
        _options_source = Utils::LoadFrom::PARAM;
        Logger::info("Configuring from ROS parameter server\n"); 
    }
    
    _xbot_cfg = Utils::LoadOptions(_options_source);
}


void RosExecutor::init_load_robot()
{
    
    /* Should we run in visual mode? */
    bool visual_mode = _nh_priv.param("visual_mode", false);
    
    /* Obtain robot (if connection available) */
    if(!visual_mode)
    {
        try
        {
            _robot = XBot::RobotInterface::getRobot(_xbot_cfg);
        }
        catch(std::exception& e)
        {
            XBot::Logger::warning("Unable to communicate with robot (exception thrown: '%s'), running in visual mode\n", e.what());
            _robot.reset();
        }
        
        if(_robot)
        {
            _robot->sense();
            _robot->setControlMode(XBot::ControlMode::Position());
        }
    }
    else
    {
        Logger::info(Logger::Severity::HIGH, "Running in visual mode: no commands sent to robot\n");
    }
}

void RosExecutor::init_customize_command()
{
    if(!_robot)
    {
        return;
    }
    
    std::map<std::string, XBot::ControlMode> ctrl_map;
    
    std::vector<std::string> joint_blacklist;
    _xbot_cfg.get_parameter("joint_blacklist", joint_blacklist);
    
    for(auto j : joint_blacklist) 
    {
        if(!_robot->hasJoint(j))
        {
            throw std::runtime_error("Joint blacklist contains non existing joint '" + j + "'");
        }
        
        ctrl_map[j] = XBot::ControlMode::Idle();
    }
    
    std::vector<std::string> velocity_whitelist;
    _xbot_cfg.get_parameter("velocity_whitelist", velocity_whitelist);
    
    for(auto j : velocity_whitelist)
    {
        if(!_robot->hasJoint(j))
        {
            throw std::runtime_error("Velocity whitelist contains non existing joint '" + j + "'");
        }
        
        ctrl_map[j] = XBot::ControlMode::Position() + XBot::ControlMode::Velocity();
    }
    
    _robot->setControlMode(ctrl_map);
}

void RosExecutor::init_load_model()
{
    /* Get model */
    _model = XBot::ModelInterface::getModel(_xbot_cfg);
    
    /* Initialize to home or to current robot state */
    reset_model_state();
}

void RosExecutor::reset_model_state()
{
    
    if(_robot)
    {
        auto msg = ros::topic::waitForMessage<xbot_msgs::JointState>("xbotcore/joint_states", ros::Duration(1.0));
        if(!msg || msg->name.size() == 0)
        {
            throw std::runtime_error("Unable to get current joint states from xbotcore");
        }
        
        XBot::JointNameMap qref;
        for(auto j : _robot->getEnabledJointNames())
        {
            auto it = std::find(msg->name.begin(), msg->name.end(), j);
            if(it == msg->name.end())
            {
                throw std::runtime_error("Unable to get current joint state for joint '" + j + "'");
            }
            int idx = std::distance(msg->name.begin(), it);
            qref[j] = msg->position_reference.at(idx);
        }
        _model->setJointPosition(qref);
        _model->update();
    }  
    else
    {
        Eigen::VectorXd qhome;
        _model->getRobotState("home", qhome);
        _model->setJointPosition(qhome);
        _model->update();
    }
        
}


void RosExecutor::init_load_torque_offset()
{
    /* Get torque offsets if available */
    _tau_offset.setZero(_model->getJointNum());
    XBot::JointNameMap tau_off_map;
    if(_xbot_cfg.get_parameter("torque_offset", tau_off_map))
    {
        _model->mapToEigen(tau_off_map, _tau_offset);
    }
}

void RosExecutor::init_load_world_frame()
{
    
    /* If the world pose is available from TF, use it */
    bool set_world_from_tf = _nh_priv.param("set_world_from_param", false);
    Eigen::Affine3d w_T_fb;
    if(_model->isFloatingBase() && 
        set_world_from_tf && 
        _xbot_cfg.get_parameter("floating_base_pose", w_T_fb))
    {
        _model->setFloatingBasePose(w_T_fb);
        _model->update();
    }
}

CartesianInterfaceImpl::Ptr RosExecutor::load_controller(std::string impl_name, 
                                                   ProblemDescription ik_problem)
{
    CartesianInterfaceImpl::Ptr impl;
    
    std::string path_to_shared_lib = XBot::Utils::FindLib("libCartesian" + impl_name + ".so", "LD_LIBRARY_PATH");
    if (path_to_shared_lib == "") 
    {
        throw std::runtime_error("libCartesian" + impl_name + ".so must be listed inside LD_LIBRARY_PATH");
    }
    
    impl = SoLib::getFactoryWithArgs<CartesianInterfaceImpl>(path_to_shared_lib, 
                                                             impl_name + "Impl", 
                                                             _model, ik_problem);
    
    if(!impl)
    {
        throw std::runtime_error("Unable to load solver '" + impl_name + "'");
    }
    else
    {
        XBot::Logger::success("Loaded solver '%s'\n", impl_name.c_str());
    }
    
    return impl;
    
    
}



bool RosExecutor::loader_callback(cartesian_interface::LoadControllerRequest& req,
                                                   cartesian_interface::LoadControllerResponse& res)
{
    /* Construct problem description */
    ProblemDescription ik_prob;
    
    if(!req.problem_description_string.empty()) // first look if problem was passed as string
    {
        ik_prob = ProblemDescription(YAML::Load(req.problem_description_string), 
                                     _model);
        res.message += "Problem description taken from string -- ";
    }
    else if(!req.problem_description_name.empty()) // then, look if it was passed by name 
    {
        auto ik_yaml = Utils::LoadProblemDescription(_options_source, 
                                                     req.problem_description_name);
        
        ik_prob = ProblemDescription(ik_yaml, _model);
        
        res.message += "Problem description taken by name -- ";
    }
    else // use problem_description
    {
        auto ik_yaml = Utils::LoadProblemDescription(_options_source);
        
        ik_prob = ProblemDescription(ik_yaml, _model);
        
        res.message += "Default problem description will be used -- ";
    }
    
    
    /* Find requested controller */
    auto ik_it = _impl_map.find(req.controller_name);
    
    if(ik_it == _impl_map.end()) // controller does not exist
    {
        // this throws on error
        _current_impl = load_controller(req.controller_name, ik_prob);
        
        // add controller to impl map
        _impl_map[req.controller_name] = _current_impl;
        
        res.message += "Required controller will be loaded";
    }
    else if(req.force_reload) // controller exists but force reload is true
    {
        Logger::info(Logger::Severity::HIGH, "Requested controller is being reloaded.. \n");
        _current_impl = load_controller(req.controller_name, ik_prob);
        
        _zombies.push_back(ik_it->second);
        _impl_map.at(req.controller_name) = _current_impl;
        
        res.message += "Required controller has been found (will be reloaded)";
    }
    else // controller exists and force reload is false
    {
        Logger::info(Logger::Severity::HIGH, "Requested controller is already loaded\n");
        _current_impl = ik_it->second;
        
        res.message += "Required controller already loaded";
    }
    
    
    // enable otg
    _current_impl->enableOtg(_period);
    
    // reset solver
    _current_impl->reset(_time);
    
    // reload ros api
    load_ros_api();
    
    // notify controller has changed
    std_msgs::Empty msg;
    _ctrl_changed_pub.publish(msg);

    /* If we arrive here, the controller was successfully loaded */
    res.success = true;
    XBot::Logger::success(Logger::Severity::HIGH, "Successfully loaded %s\n", req.controller_name.c_str());
    
    return true;
}

void RosExecutor::load_ros_api()
{
    RosServerClass::Options opt;
    opt.ros_namespace = _nh.getNamespace();
    _xbot_cfg.get_parameter("tf_prefix", opt.tf_prefix);
    _ros_api.reset();
    _ros_api = std::make_shared<RosServerClass>(_current_impl, _model, opt);
}

void RosExecutor::init_create_loop_timer()
{
    _period = 1.0 / _nh_priv.param("rate", 100.0);
    _loop_timer = _nh.createTimer(ros::Duration(_period), 
                                  &RosExecutor::timer_callback, 
                                  this, false, false);
    _time = 0.0;
    
    cartesian_interface::LoadControllerRequest load_ctrl_req;
    cartesian_interface::LoadControllerResponse load_ctrl_res;
    _xbot_cfg.get_parameter("solver", load_ctrl_req.controller_name);
    
    loader_callback(load_ctrl_req, load_ctrl_res);
    
}

void RosExecutor::spin()
{
    _loop_timer.start();
    
    Logger::info(Logger::Severity::HIGH,
                 "%s: started looping @%.1f Hz\n", ros::this_node::getName().c_str(), 1./_period);
    
    ros::spin();
}

void RosExecutor::timer_callback(const ros::TimerEvent& timer_ev)
{
    
    /* Update sensors */
    if(_robot)
    {
        _robot->sense(false);
        _model->syncFrom_no_update(*_robot, XBot::Sync::Sensors, XBot::Sync::Effort);
        _model->getJointEffort(_tau);
        _tau += _tau_offset;
        _model->setJointEffort(_tau);
        _model->update();
    }
    
    using namespace std::chrono;
    
    /* Update references from ros */
    auto tic_ros = high_resolution_clock::now();
    _ros_api->run();
    auto toc_ros = high_resolution_clock::now();
    
    /* Solve ik */
    auto tic =  high_resolution_clock::now();
    if(!_current_impl->update(_time, _period))
    {
        return;
    }
    auto toc =  high_resolution_clock::now();
    
    /* Integrate solution */
    _model->getJointPosition(_q);
    _model->getJointVelocity(_qdot);
    
    _q += _period * _qdot;
    
    _model->setJointPosition(_q);
    _model->update();
    
    if(_robot)
    {
        _robot->setReferenceFrom(*_model, Sync::Position, Sync::Velocity);
        _robot->move();
    }


    /* logging */
    _logger->add("q", _q);
    _logger->add("qdot", _qdot);
    
    /* Update time and sleep */
    _time += _period;
    double run_time_us = std::chrono::duration_cast<std::chrono::microseconds>(toc-tic).count();
    _logger->add("run_time", run_time_us);
    double ros_time_us = std::chrono::duration_cast<std::chrono::microseconds>(toc_ros-tic_ros).count();
    _logger->add("ros_time", ros_time_us);
}

bool RosExecutor::reset_callback(std_srvs::TriggerRequest& req, 
                                 std_srvs::TriggerResponse& res)
{
    reset_model_state();
    
    _current_impl->reset(_time);
    
    XBot::JointNameMap q_map;
    _model->getJointPosition(q_map);
    _current_impl->setReferencePosture(q_map);
    
    Logger::info(Logger::Severity::HIGH, "Reset was performed successfully\n");
    
    return true;
}


void RosExecutor::world_frame_to_param()
{
    /* Upload floating base pose to parameter server */
    Eigen::Affine3d w_T_fb;
    _model->getFloatingBasePose(w_T_fb);
    
    std::vector<double> linear(3);
    Eigen::Vector3d::Map(linear.data()) = w_T_fb.translation();
    _nh.setParam("floating_base_pose/linear", linear);
    
    
    std::vector<double> angular(4);
    Eigen::Vector4d::Map(angular.data()) = Eigen::Quaterniond(w_T_fb.linear()).coeffs();
    _nh.setParam("floating_base_pose/angular", angular);
}

RosExecutor::~RosExecutor()
{

    world_frame_to_param();
    
    _logger->flush();
}








