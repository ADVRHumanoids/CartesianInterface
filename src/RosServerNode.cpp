#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <std_msgs/Empty.h>

#include <XBotInterface/RobotInterface.h>
#include <XBotInterface/Utils.h>
#include <XBotInterface/SoLib.h>


#include <cartesian_interface/ros/RosServerClass.h>
#include <cartesian_interface/problem/ProblemDescription.h>
#include <cartesian_interface/LoadController.h>
#include <cartesian_interface/utils/LoadConfig.h>
#include <cartesian_interface/CartesianInterfaceImpl.h>


#define BOOST_RESULT_OF_USE_DECLTYPE
#include <boost/function.hpp>
#include <chrono>

using namespace XBot::Cartesian;

/* Global variables to be used in callbacks */
XBot::ModelInterface::Ptr g_model;
ProblemDescription * g_problem;
std::map<std::string, CartesianInterfaceImpl::Ptr>  g_impl_map;
std::vector<CartesianInterfaceImpl::Ptr> g_zombies;
CartesianInterfaceImpl::Ptr g_current_impl;
RosServerClass::Ptr * g_ros_ptrptr;
double g_period;
double *g_time;
ros::Publisher * g_reset_pub;
Utils::LoadFrom g_options_source;

/* Handle control modes */
void set_blacklist(XBot::RobotInterface::Ptr robot, const XBot::ConfigOptions& xbot_cfg);

/* Load controller */
CartesianInterfaceImpl::Ptr load_controller(std::string impl_name, 
                                            XBot::ModelInterface::Ptr model, 
                                            ProblemDescription ik_prob);

/* Loader function */
bool loader_callback(cartesian_interface::LoadControllerRequest&  req, 
                     cartesian_interface::LoadControllerResponse& res);


int main(int argc, char **argv){
    
    using std::chrono::high_resolution_clock;
    
    auto logger = XBot::MatLogger::getLogger("/tmp/cartesian_ros_node_log");
    
    
    
    /* Init ROS node */
    ros::init(argc, argv, "xbot_cartesian_server");
    ros::NodeHandle nh("cartesian");
    ros::NodeHandle nh_private("~");
    
    
    /* Should we run in visual mode? */
    bool visual_mode = nh.param("visual_mode", false);
    
    /* Obtain xbot config object */
    bool use_xbot_config = nh_private.param("use_xbot_config", false);
    XBot::ConfigOptions xbot_cfg;
    YAML::Node problem_yaml;
    
    if(use_xbot_config)
    {
        g_options_source = Utils::LoadFrom::CONFIG;
        Logger::info("Configuring from file %s\n", XBot::Utils::getXBotConfig().c_str());
    }
    else
    {
        g_options_source = Utils::LoadFrom::PARAM;
        Logger::info("Configuring from ROS parameter server\n"); 
    }
    
    xbot_cfg = Utils::LoadOptions(g_options_source);
    problem_yaml = Utils::LoadProblemDescription(g_options_source);
    
    /* Obtain robot (if connection available) */
    XBot::RobotInterface::Ptr robot;
    if(!visual_mode)
    {
        try
        {
            robot = XBot::RobotInterface::getRobot(xbot_cfg);
        }
        catch(std::exception& e)
        {
            XBot::Logger::warning("Unable to communicate with robot (exception thrown: '%s'), running in visual mode\n", e.what());
            robot.reset();
        }
        
        if(robot)
        {
            robot->sense();
            robot->setControlMode(XBot::ControlMode::Position());
            set_blacklist(robot, xbot_cfg);
        }
    }
    else
    {
        Logger::info(Logger::Severity::HIGH, "Running in visual mode: no commands sent to robot\n");
    }
    
    /* Get model, initialize to home */
    auto model = XBot::ModelInterface::getModel(xbot_cfg);
    Eigen::VectorXd qhome;
    model->getRobotState("home", qhome);
    model->setJointPosition(qhome);
    model->update();
    
    if(robot)
    {
        model->syncFrom(*robot, 
                        XBot::Sync::Position, 
                        XBot::Sync::MotorSide, 
                        XBot::Sync::Impedance);
    }
    
    /* Get tf_prefix */
    std::string tf_prefix = nh_private.param<std::string>("tf_prefix", "ci");
    if(tf_prefix == "null")
    {
        tf_prefix = "";
    }
    
    /* Change world frame to the specified link */
    std::string world_frame;
    if(xbot_cfg.get_parameter("world_frame", world_frame) && world_frame != "")
    {
        Eigen::Affine3d w_T_l;
        if(model->getPose(world_frame, w_T_l))
        {
            model->setFloatingBasePose(w_T_l.inverse());
            model->update();
            Logger::success(Logger::Severity::HIGH, "World frame set to link %s\n", world_frame.c_str());
        }
        else
        {
            Logger::warning("Unable to set world frame to link %s (undefined frame)\n", world_frame.c_str());
        }
    }
    
    /* If the world pose is available from TF, use it */
    {
        bool set_world_from_tf = nh_private.param("set_world_from_param", false);
        Eigen::Affine3d w_T_fb;
        if(model->isFloatingBase() && 
            set_world_from_tf && 
            xbot_cfg.get_parameter("floating_base_pose", w_T_fb))
        {
            model->setFloatingBasePose(w_T_fb);
            model->update();
        }
    }
    
    /* Load IK problem and solver */
    std::string impl_name;
    if(!xbot_cfg.get_parameter("solver", impl_name))
    {
        throw std::runtime_error("solver parameter missing");
    }
    Logger::info("Cartesian solver %s will be loaded\n", impl_name.c_str());
    
    ProblemDescription ik_problem(problem_yaml, model);
    
    /* Load controller from string */
    g_current_impl = load_controller(impl_name, model, ik_problem);
    g_impl_map[impl_name] = g_current_impl;
    if(!g_current_impl)
    {
        exit(1);
    }
    
    auto reset_pub = nh.advertise<std_msgs::Empty>("changed_controller_event", 1);
    g_reset_pub = &reset_pub;
    std_msgs::Empty msg;
    g_reset_pub->publish(msg);
    
    /* Obtain class to expose ROS API */
    XBot::Cartesian::RosServerClass::Options opt;
    opt.tf_prefix = tf_prefix;
    auto ros_server_class = std::make_shared<XBot::Cartesian::RosServerClass>(g_current_impl, model, opt);
    
    g_model = model;
    g_impl_map[impl_name] = g_current_impl;
    g_problem = &ik_problem;
    g_ros_ptrptr = &ros_server_class;
    
    
    
    auto loader_srv = nh.advertiseService("load_controller", loader_callback);
    
    
    /* Get loop frequency */
    const double freq = nh_private.param("rate", 100);
    g_period = 1.0/freq;
    g_current_impl->enableOtg(1.0/freq);
    ros::Rate loop_rate(freq);
    
    Eigen::VectorXd q, qdot;
    model->getJointPosition(q);
    double time = 0.0;
    g_time = &time;
    double dt = loop_rate.expectedCycleTime().toSec();
    
    Logger::info(Logger::Severity::HIGH,
                 "%s: started looping @%.1f Hz\n", ros::this_node::getName().c_str(), freq);
    
    while(ros::ok())
    {
        /* Update sensors */
        if(robot)
        {
            robot->sense(false);
            model->syncFrom(*robot, XBot::Sync::Sensors, XBot::Sync::Effort);
        }
        
        /* Update references from ros */
        ros::spinOnce();
        auto tic_ros = high_resolution_clock::now();
        ros_server_class->run();
        auto toc_ros = high_resolution_clock::now();
        
        /* Solve ik */
        auto tic =  high_resolution_clock::now();
        g_current_impl->update(time, dt);
        auto toc =  high_resolution_clock::now();
        
        /* Integrate solution */
        model->getJointPosition(q);
        model->getJointVelocity(qdot);
        
        q += dt * qdot;


        
        model->setJointPosition(q);
        model->update();
        
        if(robot)
        {
            robot->setReferenceFrom(*model);
            robot->move();
        }


        /* logging */
        logger->add("q", q);
        logger->add("qdot", qdot);
        logger->add("dq", qdot*dt);
        
        /* Update time and sleep */
        time += dt;
        logger->add("loop_time", loop_rate.cycleTime().toSec()*1e6);
        double run_time_us = std::chrono::duration_cast<std::chrono::microseconds>(toc-tic).count();
        logger->add("run_time", run_time_us);
        double ros_time_us = std::chrono::duration_cast<std::chrono::microseconds>(toc_ros-tic_ros).count();
        logger->add("ros_time", ros_time_us);
        loop_rate.sleep();
        
        
    }
    
    /* Upload floating base pose to parameter server */
    Eigen::Affine3d w_T_fb;
    model->getFloatingBasePose(w_T_fb);
    
    std::vector<double> linear(3);
    Eigen::Vector3d::Map(linear.data()) = w_T_fb.translation();
    nh.setParam("floating_base_pose/linear", linear);
    
    
    std::vector<double> angular(4);
    Eigen::Vector4d::Map(angular.data()) = Eigen::Quaterniond(w_T_fb.linear()).coeffs();
    nh.setParam("floating_base_pose/angular", angular);
    
    /* Flush log */
    logger->flush();
    
    return 0;
    
}


bool loader_callback(cartesian_interface::LoadControllerRequest& req, 
                     cartesian_interface::LoadControllerResponse& res)
{
    ProblemDescription ik_prob;
    /* If required, load problem description */
    if(!req.problem_description_string.empty()) // first look if problem was passed as string
    {
        ik_prob = ProblemDescription(YAML::Load(req.problem_description_string), 
                                     g_model);
    }
    else if(!req.problem_description_name.empty()) // then, look if it was passed by name 
    {
        auto ik_yaml = Utils::LoadProblemDescription(g_options_source, 
                                                     req.problem_description_name);
        
        ik_prob = ProblemDescription(ik_yaml, g_model);
    }
    else // use problem_description
    {
        auto ik_yaml = Utils::LoadProblemDescription(g_options_source);
        
        ik_prob = ProblemDescription(ik_yaml, g_model);
    }
    
    /* Find requested controller */
    auto ik_it = g_impl_map.find(req.controller_name);
    CartesianInterfaceImpl::Ptr current_impl;
    
    if(ik_it == g_impl_map.end()) // controller does not exist
    {
        current_impl = load_controller(req.controller_name, g_model, ik_prob);
        
        if(current_impl)
        {
            g_impl_map[req.controller_name] = current_impl;
        }
    }
    else if(req.force_reload) // controller exists but force reload is true
    {
        Logger::info(Logger::Severity::HIGH, "Requested controller is being reloaded.. \n");
        current_impl = load_controller(req.controller_name, g_model, ik_prob);
        
        if(current_impl)
        {
            g_zombies.push_back(ik_it->second);
            g_impl_map.at(req.controller_name) = current_impl;
        }
    }
    else // controller exists and force reload is false
    {
        Logger::info(Logger::Severity::HIGH, "Requested controller is already loaded\n");
        current_impl = ik_it->second;
    }
    
    if(current_impl)
    {
        
        current_impl->enableOtg(g_period);
        current_impl->reset(*g_time);
        res.success = true;
        res.message = "Successfully loaded controller";
        g_ros_ptrptr->reset();
        XBot::Cartesian::RosServerClass::Options opt;
        *g_ros_ptrptr = std::make_shared<XBot::Cartesian::RosServerClass>(current_impl, g_model, opt);
        g_current_impl = current_impl;
        
        std_msgs::Empty msg;
        g_reset_pub->publish(msg);
        
        *g_problem = ik_prob;
        
        XBot::Logger::success(Logger::Severity::HIGH, "Successfully loaded %s\n", req.controller_name.c_str());
        
    }
    else
    {
        res.success = false;
        res.message = "Unable to load controller";
    }
    
    
    return true;
};


void set_blacklist ( XBot::RobotInterface::Ptr robot, const XBot::ConfigOptions& xbot_cfg )
{
    std::map<std::string, XBot::ControlMode> ctrl_map;
    
    std::vector<std::string> joint_blacklist;
    xbot_cfg.get_parameter("joint_blacklist", joint_blacklist);
    for ( auto j : joint_blacklist ) {
        if(!robot->hasJoint(j))
        {
            throw std::runtime_error("Joint blacklist contains non existing joint " + j);
        }
        ctrl_map[j] = XBot::ControlMode::Idle();
    }
    
    std::vector<std::string> velocity_whitelist;
    xbot_cfg.get_parameter("velocity_whitelist", velocity_whitelist);
    for ( auto j : velocity_whitelist ) {
        if(!robot->hasJoint(j))
        {
            throw std::runtime_error("Velocity whitelist contains non existing joint " + j);
        }
        ctrl_map[j] = XBot::ControlMode::Position() + XBot::ControlMode::Velocity();
    }
    
    robot->setControlMode(ctrl_map);
}

CartesianInterfaceImpl::Ptr load_controller(std::string impl_name, 
                                            XBot::ModelInterface::Ptr model, 
                                            ProblemDescription ik_problem)
{
    CartesianInterfaceImpl::Ptr impl;
    
    std::string path_to_shared_lib = XBot::Utils::FindLib("libCartesian" + impl_name + ".so", "LD_LIBRARY_PATH");
    if (path_to_shared_lib == "") 
    {
        throw std::runtime_error("libCartesian" + impl_name + ".so must be listed inside LD_LIBRARY_PATH");
    }
    
    impl = SoLib::getFactoryWithArgs<XBot::Cartesian::CartesianInterfaceImpl>(path_to_shared_lib, 
                                                                              impl_name + "Impl", 
                                                                              model, ik_problem);
    
    if(!impl)
    {
        XBot::Logger::error("Unable to load solver '%s'\n", impl_name.c_str());
    }
    else
    {
        XBot::Logger::success("Loaded solver '%s'\n", impl_name.c_str());
    }
    
    return impl;
}

