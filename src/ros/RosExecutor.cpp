#include <cartesian_interface/ros/RosExecutor.h>
#include <cartesian_interface/utils/LoadConfig.h>
#include "utils/RosUtils.h"

#include <rclcpp/wait_for_message.hpp>

using namespace XBot::Cartesian;

RosExecutor::RosExecutor(std::string ns):
    _visual_mode(false),
    _fb_feedback(false)
{

    /* Create logger */
    MatLogger2::Options logger_opt;
    logger_opt.default_buffer_size = 1e5;
    _logger = MatLogger2::MakeLogger("/tmp/ros_executor_log", logger_opt);
    _logger->set_buffer_mode(VariableBuffer::Mode::circular_buffer);


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
    /* Create node*/
    _node = rclcpp::Node::make_shared("ros_server_node",
                                      rclcpp::NodeOptions()
                                          .allow_undeclared_parameters(true)
                                          .automatically_declare_parameters_from_overrides(true));

    _cnode = _node->create_sub_node("cartesian");

    _prnode = _node->create_sub_node(_node->get_name());

    RCLCPP_INFO(_node->get_logger(), "hello");
    RCLCPP_INFO(_cnode->get_logger(), "hello");
    RCLCPP_INFO(_prnode->get_logger(), "hello");

    // TBD declare parameters

    /* Broadcast a notification whenever a new controller is loaded */
    _ctrl_changed_pub = _cnode->create_publisher<std_msgs::msg::Empty>(
        "changed_controller_event", 1);

    /* Load controller service */
    _loader_srv = ::create_service<LoadController>(_cnode,
                                                   "load_controller",
                                                   &RosExecutor::loader_callback, this);

    /* Load reset service */
    _reset_srv = ::create_service<Trigger>(_cnode,
                                           "reset",
                                           &RosExecutor::reset_callback, this);

    _reset_joints_srv = ::create_service<ResetJoints>(_cnode,
                                                      "reset_joints",
                                                      &RosExecutor::reset_joints_callback, this);

    /* Floating base override topic */
    _fb_sub = ::create_subscription<PoseStamped>(_cnode,
                                                 "base_pose_override",
                                                 &RosExecutor::floating_base_pose_callback,
                                                 this,
                                                 1);
}

void RosExecutor::init_load_config()
{
    /* Obtain xbot config object */
    bool use_xbot_config = _node->get_parameter_or("use_xbot_config", false);

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

    _xbot_cfg_robot = Utils::LoadOptions(_options_source, _node);

    try
    {
        _xbot_cfg = Utils::LoadOptions(_options_source, _node, "model_description/");
    }
    catch(std::exception& e)
    {
        _xbot_cfg = _xbot_cfg_robot;
    }

    _period = 1.0 / _node->get_parameter_or("rate", 100.0);

}

void RosExecutor::init_load_robot()
{
    /* Should we run in visual mode? */
    _visual_mode = _node->get_parameter_or("visual_mode", false);

    try
    {
        _robot = XBot::RobotInterface::getRobot(_xbot_cfg_robot);
    }
    catch(std::exception& e)
    {
        XBot::Logger::warning("Unable to communicate with robot (exception thrown: '%s'), running in visual mode\n", e.what());
        _robot.reset();
        _visual_mode = true;
    }

    if(_robot)
    {
        _robot->sense();
        _robot->setControlMode(XBot::ControlMode::Position());
    }

    if(_robot && _robot->model().isFloatingBase())
    {
        _fb_pub = _cnode->create_publisher<geometry_msgs::msg::Twist>(
            "floating_base_velocity", 1);
    }


    /* Obtain robot (if connection available) */
    if(!_visual_mode)
    {
    }
    else
    {
        Logger::info(Logger::Severity::HIGH,
                     "Running in visual mode: no commands sent to robot\n");
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

        // if joint blacklisted, only velocity
        if(std::find(joint_blacklist.begin(), joint_blacklist.end(), j) !=
                joint_blacklist.end())
        {
            ctrl_map[j] = XBot::ControlMode::Velocity();
        }
        // else, also position
        else
        {
            ctrl_map[j] = XBot::ControlMode::Position() + XBot::ControlMode::Velocity();
        }


    }

    _robot->setControlMode(ctrl_map);
}

void RosExecutor::init_load_model()
{
    /* Get model */
    _model = XBot::ModelInterface::getModel(_xbot_cfg);

    /* Initialize to home or to current robot state */
    reset_model_state();

    /* Context for CI */
    _ctx = std::make_shared<Context>(
        std::make_shared<Parameters>(_period),
        _model);

}

void RosExecutor::reset_model_state()
{

    if(_robot)
    {
        auto timeout = _node->get_clock()->now() + 10s;

        while(!_robot->sense(false, true))
        {

            if(_node->get_clock()->now() > timeout)
            {
                throw std::runtime_error("unable to receive state from robot");
            }

            _node->get_clock()->sleep_for(100ms);
        }

        XBot::JointNameMap qref;
        _robot->getPositionReference(qref);
        _model->setJointPosition(qref);
        _model->update();
    }
    // else if(_node->has_parameter("home"))
    // {
    //     std::map<std::string, double> joint_map;
    //     _node->get_parameter("home", joint_map);

    //     XBot::JointNameMap qref(joint_map.begin(),
    //                             joint_map.end());


    //     _model->setJointPosition(qref);
    //     _model->update();
    // }
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

    if(!_model->isFloatingBase())
    {
        return;
    }

    /* If the world pose is available from TF, use it */
    bool set_world_from_tf = _node->get_parameter_or("set_world_from_param", false);
    Eigen::Affine3d w_T_fb;
    if(_model->isFloatingBase() &&
        set_world_from_tf &&
        _xbot_cfg.get_parameter("floating_base_pose", w_T_fb))
    {
        _model->setFloatingBasePose(w_T_fb);
        _model->update();
    }

    /* If set_world_from_param is false, and a world_frame_link is defined... */
    std::string world_frame_link;
    if(!set_world_from_tf && _xbot_cfg.get_parameter("world_frame_link", world_frame_link))
    {
        Eigen::Affine3d fb_T_l;
        std::string floating_base_link;
        _model->getFloatingBaseLink(floating_base_link);
        if(!_model->getPose(world_frame_link, floating_base_link, fb_T_l))
        {
            throw std::runtime_error("World frame link '" + world_frame_link + "' is undefined");
        }

        _model->setFloatingBasePose(fb_T_l.inverse());
        _model->update();
    }
}

CartesianInterfaceImpl::Ptr RosExecutor::load_controller(std::string impl_name,
                                                   ProblemDescription ik_problem)
{
    auto impl = CartesianInterfaceImpl::MakeInstance(impl_name,
                                                     ik_problem, _ctx);

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



bool RosExecutor::loader_callback(LoadController::Request::ConstSharedPtr req,
                                  LoadController::Response::SharedPtr res)
{
    /* Construct problem description */
    ProblemDescription ik_prob;

    if(!req->problem_description_string.empty()) // first look if problem was passed as string
    {
        ik_prob = ProblemDescription(YAML::Load(req->problem_description_string),
                                     _ctx);
        res->message += "Problem description taken from string -- ";
    }
    else if(!req->problem_description_name.empty()) // then, look if it was passed by name
    {
        auto ik_yaml = Utils::LoadProblemDescription(_options_source,
                                                     _node,
                                                     req->problem_description_name);

        ik_prob = ProblemDescription(ik_yaml, _ctx);

        res->message += "Problem description taken by name -- ";
    }
    else // use problem_description
    {
        auto ik_yaml = Utils::LoadProblemDescription(_options_source, _node);

        ik_prob = ProblemDescription(ik_yaml, _ctx);

        res->message += "Default problem description will be used -- ";
    }


    /* Find requested controller */
    auto ik_it = _impl_map.find(req->controller_name);

    if(ik_it == _impl_map.end()) // controller does not exist
    {
        // this throws on error
        _current_impl = load_controller(req->controller_name, ik_prob);

        // add controller to impl map
        _impl_map[req->controller_name] = _current_impl;

        res->message += "Required controller will be loaded";
    }
    else if(req->force_reload) // controller exists but force reload is true
    {
        Logger::info(Logger::Severity::HIGH, "Requested controller is being reloaded.. \n");
        _current_impl = load_controller(req->controller_name, ik_prob);

        _zombies.push_back(ik_it->second);
        _impl_map.at(req->controller_name) = _current_impl;

        res->message += "Required controller has been found (will be reloaded)";
    }
    else // controller exists and force reload is false
    {
        Logger::info(Logger::Severity::HIGH, "Requested controller is already loaded\n");
        _current_impl = ik_it->second;

        res->message += "Required controller already loaded";
    }

    // enable otg
    if(_node->get_parameter_or("enable_otg", true))
    {
        _current_impl->enableOtg(_period);
    }

    // reset solver
    _current_impl->reset(_time);

    // reload ros api
    load_ros_api();

    // notify controller has changed, unless we are at system startup
    static bool first_time_controller_loaded = true;

    if(!first_time_controller_loaded)
    {
        std_msgs::msg::Empty msg;
        _ctrl_changed_pub->publish(msg);
    }

    first_time_controller_loaded = false;

    /* If we arrive here, the controller was successfully loaded */
    res->success = true;
    XBot::Logger::success(Logger::Severity::HIGH, "Successfully loaded %s\n", req->controller_name.c_str());

    return true;
}

void RosExecutor::load_ros_api()
{
    RosServerClass::Options opt;
    opt.ros_namespace = _node->get_namespace();
    _xbot_cfg.get_parameter("tf_prefix", opt.tf_prefix);
    _ros_api.reset();
    _ros_api = std::make_shared<RosServerClass>(_current_impl, opt, _cnode);
}

void RosExecutor::init_create_loop_timer()
{

    _loop_timer = _node->create_timer(std::chrono::duration<double>(_period),
                        [this]()
                        {
                            timer_callback();
                        });

    _time = 0.0;

    auto load_ctrl_req = std::make_shared<LoadController::Request>();
    auto load_ctrl_res = std::make_shared<LoadController::Response>();

    if(!_xbot_cfg.get_parameter("solver", load_ctrl_req->controller_name))
    {
        throw std::runtime_error("'solver' parameter missing");
    }

    loader_callback(load_ctrl_req, load_ctrl_res);

}

void RosExecutor::spin()
{
    Logger::info(Logger::Severity::HIGH,
                 "%s: started looping @%.1f Hz\n", _node->get_name(), 1./_period);

    rclcpp::spin(_node);
}

CartesianInterfaceImpl& RosExecutor::solver()
{
    return *_current_impl;
}

const XBot::ModelInterface& RosExecutor::model()
{
    return *_model;
}

void RosExecutor::timer_callback()
{

    /* Update sensors */
    if(_robot && !_visual_mode)
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

    if(_model->isFloatingBase())
    {
        publish_fb_cmd_vel();
    }
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
    _model->getJointAcceleration(_qddot);

    _q += _period * _qdot + 0.5 * std::pow(_period, 2) * _qddot;
    _qdot += _period * _qddot;

    _model->setJointPosition(_q);
    _model->setJointVelocity(_qdot);

    // override floating base from topic
    // note: done by main node/executor
    // is this ok ?

    _model->update();

    if(_robot && !_visual_mode)
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

void RosExecutor::publish_fb_cmd_vel()
{
    if(!_fb_pub)
    {
        return;
    }

    // get floating base state from solution
    Eigen::Affine3d T;
    Eigen::Vector6d v;

    _model->getFloatingBasePose(T);
    _model->getFloatingBaseTwist(v);

    // turn twist to local coordinates
    v.head<3>() = T.linear().transpose()*v.head<3>();
    v.tail<3>() = T.linear().transpose()*v.tail<3>();

    // fill in msg
    geometry_msgs::msg::Twist cmd = tf2::toMsg(v);

    // publish
    _fb_pub->publish(cmd);
}

bool RosExecutor::reset_callback(Trigger::Request::ConstSharedPtr req,
                                 Trigger::Response::SharedPtr res)
{
    reset_model_state();

    _current_impl->reset(_time);

    XBot::JointNameMap q_map;
    _model->getJointPosition(q_map);
    _current_impl->setReferencePosture(q_map);

    Logger::info(Logger::Severity::HIGH, "Reset was performed successfully\n");

    return true;
}

bool RosExecutor::reset_joints_callback(ResetJoints::Request::ConstSharedPtr req,
                                        ResetJoints::Response::SharedPtr res)
{
    XBot::JointNameMap q_map;
    _model->getJointPosition(q_map);

    for(int i = 0; i < req->joint_names.size(); i++)
    {
        auto it = q_map.find(req->joint_names[i]);
        if(it != q_map.end())
            it->second = req->joint_values[i];
    }
    _model->setJointPosition(q_map);
    _model->update();

    _current_impl->reset(_time);

    _current_impl->setReferencePosture(q_map);

    return true;
}

void RosExecutor::floating_base_pose_callback(PoseStamped::ConstSharedPtr msg)
{
    if(!_fb_feedback)
    {
        return;
    }

    Eigen::Affine3d w_T_fb;

    tf2::fromMsg(msg->pose, w_T_fb);

    _model->setFloatingBasePose(w_T_fb);
}

RosExecutor::~RosExecutor()
{
}








