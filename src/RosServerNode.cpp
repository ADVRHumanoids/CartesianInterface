#include <ros/ros.h>

#include <XBotInterface/RobotInterface.h>
#include <XBotInterface/Utils.h>
#include <XBotInterface/SoLib.h>

#include <cartesian_interface/ros/RosServerClass.h>
#include <cartesian_interface/ProblemDescription.h>
#include <cartesian_interface/LoadController.h>

#define BOOST_RESULT_OF_USE_DECLTYPE
#include <boost/function.hpp>



using namespace XBot::Cartesian;

XBot::ModelInterface::Ptr __g_model;
ProblemDescription * __g_problem;
CartesianInterface::Ptr * __g_impl_ptrptr;
RosServerClass::Ptr * __g_ros_ptrptr;

/* Loader function */
bool loader_callback(cartesian_interface::LoadControllerRequest&  req, 
                    cartesian_interface::LoadControllerResponse& res)
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
        
    }
    else
    {
        res.success = false;
        res.message = "Unable to load controller";
    }
    
    
    return true;
};

int main(int argc, char **argv){
    
    auto logger = XBot::MatLogger::getLogger("/tmp/cartesian_ros_node_log");
    
    /* By default, MID verbosity level */
    XBot::Logger::SetVerbosityLevel(XBot::Logger::Severity::MID);
    
    /* Init ROS node */
    ros::init(argc, argv, "xbot_cartesian_server");
    ros::NodeHandle nh;
    
    XBot::RobotInterface::Ptr robot;
    
    std::string param_name = "/xbotcore/cartesian/visual_mode";
    bool visual_mode = false;
    
    if(nh.hasParam(param_name))
    {
        nh.getParam(param_name, visual_mode);
    }
    
    if(!visual_mode)
    {
        robot = XBot::RobotInterface::getRobot(XBot::Utils::getXBotConfig());
        robot->sense();
    }
    
    /* Get model, initialize to home */
    auto model = XBot::ModelInterface::getModel(XBot::Utils::getXBotConfig());
    Eigen::VectorXd qhome;
    model->getRobotState("home", qhome);
    model->setJointPosition(qhome);
    model->update();
    
    if(robot)
    {
        model->syncFrom(*robot, XBot::Sync::Position, XBot::Sync::MotorSide, XBot::Sync::Impedance);
    }
    

    /* Load IK problem and solver */
    auto yaml_file = YAML::LoadFile(XBot::Utils::getXBotConfig());
    ProblemDescription ik_problem(yaml_file["CartesianInterface"]["problem_description"], model);
    std::string impl_name = yaml_file["CartesianInterface"]["solver"].as<std::string>();
    
    auto sot_ik_solver = SoLib::getFactoryWithArgs<XBot::Cartesian::CartesianInterface>("Cartesian" + impl_name + ".so", 
                                                                                        impl_name + "Impl", 
                                                                                        model, ik_problem);
    
    if(!sot_ik_solver)
    {
        XBot::Logger::error("Unable to load solver %s \n", impl_name.c_str());
        exit(1);
    }
    
    /* Obtain class to expose ROS API */
    auto ros_server_class = std::make_shared<XBot::Cartesian::RosServerClass>(sot_ik_solver, model);
    
    __g_model = model;
    __g_impl_ptrptr = &sot_ik_solver;
    __g_problem = &ik_problem;
    __g_ros_ptrptr = &ros_server_class;
    
    
    auto loader_srv = nh.advertiseService("/xbotcore/cartesian/load_controller", loader_callback);
    
    ros::Rate loop_rate(100);
    Eigen::VectorXd q, qdot;
    model->getJointPosition(q);
    double time = 0.0;
    double dt = loop_rate.expectedCycleTime().toSec();
    
    while(ros::ok())
    {
        /* Update references from ros */
        ros::spinOnce();
        ros_server_class->run();
        
        /* Solve ik */
        sot_ik_solver->update(time, dt);
        
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
        loop_rate.sleep();
        
        
    }
    
    logger->flush();
    
    return 0;
    
}