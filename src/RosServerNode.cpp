#include <ros/ros.h>

#include <XBotInterface/RobotInterface.h>
#include <XBotInterface/Utils.h>
#include <XBotInterface/SoLib.h>
#include <RobotInterfaceROS/ConfigFromParam.h>

#include <cartesian_interface/ros/RosServerClass.h>
#include <cartesian_interface/ProblemDescription.h>
#include <cartesian_interface/LoadController.h>

#define BOOST_RESULT_OF_USE_DECLTYPE
#include <boost/function.hpp>
#include <chrono>

using namespace XBot::Cartesian;


XBot::ModelInterface::Ptr __g_model;
ProblemDescription * __g_problem;
CartesianInterface::Ptr * __g_impl_ptrptr;
RosServerClass::Ptr * __g_ros_ptrptr;

/* Loader function */
bool loader_callback(cartesian_interface::LoadControllerRequest&  req, 
                    cartesian_interface::LoadControllerResponse& res);

int main(int argc, char **argv){
    
    using std::chrono::high_resolution_clock;
    
    auto logger = XBot::MatLogger::getLogger("/tmp/cartesian_ros_node_log");
    
    /* By default, MID verbosity level */
    XBot::Logger::SetVerbosityLevel(XBot::Logger::Severity::MID);
    
    /* Init ROS node */
    ros::init(argc, argv, "xbot_cartesian_server");
    ros::NodeHandle nh("xbotcore/cartesian");
    ros::NodeHandle nh_private("~");
    
    XBot::RobotInterface::Ptr robot;
    
    std::string param_name = "visual_mode";
    bool visual_mode = false;
    
    if(nh.hasParam(param_name))
    {
        nh.getParam(param_name, visual_mode);
    }
    
    auto xbot_cfg = XBot::ConfigOptionsFromParamServer();
    
    if(!visual_mode)
    {
        try
        {
            robot = XBot::RobotInterface::getRobot(xbot_cfg);
        }
        catch(std::exception& e)
        {
            XBot::Logger::warning("Unable to communicate with robot (exception thrown: %s), running in visual mode\n", e.what());
            robot.reset();
        }
        
        if(robot)
        {
            robot->sense();
            robot->setControlMode(XBot::ControlMode::Position() + XBot::ControlMode::Velocity());
        }
    }
    
    /* Get model, initialize to home */
    auto model = XBot::ModelInterface::getModel(xbot_cfg);
    Eigen::VectorXd qhome;
    model->getRobotState("home", qhome);
    model->setJointPosition(qhome);
    model->update();
    
    if(robot)
    {
        model->syncFrom(*robot, XBot::Sync::Position, XBot::Sync::MotorSide, XBot::Sync::Impedance);
        std::cout << *model << std::endl;
    }
    
    /* Change world frame to the specified link */
    std::string world_frame;
    if(nh_private.hasParam("world_frame") && nh_private.getParam("world_frame", world_frame))
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
    
    /* Load IK problem and solver */
    std::string problem_description_string;
    if(!nh_private.hasParam("problem_description") || !nh_private.getParam("problem_description", problem_description_string))
    {
        throw std::runtime_error("problem_description private parameter missing");
    }
    
    std::string impl_name;
    if(!nh_private.hasParam("solver") || !nh_private.getParam("solver", impl_name))
    {
        throw std::runtime_error("solver private parameter missing");
    }
    
    
    auto problem_yaml = YAML::Load(problem_description_string);
    ProblemDescription ik_problem(problem_yaml, model);
    
    auto sot_ik_solver = SoLib::getFactoryWithArgs<XBot::Cartesian::CartesianInterface>("Cartesian" + impl_name + ".so", 
                                                                                        impl_name + "Impl", 
                                                                                        model, ik_problem);
    
    if(!sot_ik_solver)
    {
        XBot::Logger::error("Unable to load solver %s \n", impl_name.c_str());
        exit(1);
    }
    
    /* Obtain class to expose ROS API */
    XBot::Cartesian::RosServerClass::Options opt;
    opt.spawn_markers = false;
    auto ros_server_class = std::make_shared<XBot::Cartesian::RosServerClass>(sot_ik_solver, model, opt);
    
    __g_model = model;
    __g_impl_ptrptr = &sot_ik_solver;
    __g_problem = &ik_problem;
    __g_ros_ptrptr = &ros_server_class;
    
    
    auto loader_srv = nh.advertiseService("load_controller", loader_callback);
    
    ros::Rate loop_rate(100);
    Eigen::VectorXd q, qdot;
    model->getJointPosition(q);
    double time = 0.0;
    double dt = loop_rate.expectedCycleTime().toSec();
    
    
    while(ros::ok())
    {
        /* Update references from ros */
        ros::spinOnce();
        auto tic_ros = high_resolution_clock::now();
        ros_server_class->run();
        auto toc_ros = high_resolution_clock::now();
        
        /* Solve ik */
        auto tic =  high_resolution_clock::now();
        sot_ik_solver->update(time, dt);
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
        
        /* Update time and sleep */
        time += dt;
        logger->add("loop_time", loop_rate.cycleTime().toSec()*1e6);
        double run_time_us = std::chrono::duration_cast<std::chrono::microseconds>(toc-tic).count();
        logger->add("run_time", run_time_us);
        double ros_time_us = std::chrono::duration_cast<std::chrono::microseconds>(toc_ros-tic_ros).count();
        logger->add("ros_time", ros_time_us);
        loop_rate.sleep();
        
        
    }
    
    logger->flush();
    
    return 0;
    
}


bool loader_callback(cartesian_interface::LoadControllerRequest& req, cartesian_interface::LoadControllerResponse& res)
{
    auto tmp_ik_solver = SoLib::getFactoryWithArgs<XBot::Cartesian::CartesianInterface>(
                                                        "Cartesian" + req.controller_name + ".so", 
                                                        req.controller_name + "Impl", 
                                                        __g_model, *__g_problem);
    
    if(tmp_ik_solver)
    {
        *__g_impl_ptrptr = tmp_ik_solver;
        res.success = true;
        res.message = "Successfully loaded controller";
        __g_ros_ptrptr->reset();
        *__g_ros_ptrptr = std::make_shared<XBot::Cartesian::RosServerClass>(*__g_impl_ptrptr, __g_model);
        
        XBot::Logger::success(Logger::Severity::HIGH, "Successfully loaded %s\n", req.controller_name.c_str());
        
    }
    else
    {
        res.success = false;
        res.message = "Unable to load controller";
    }
    
    
    return true;
};
