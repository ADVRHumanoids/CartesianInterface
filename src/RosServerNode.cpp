#include <ros/ros.h>

#include <XBotInterface/RobotInterface.h>
#include <XBotInterface/Utils.h>
#include <XBotInterface/SoLib.h>

#include <CartesianInterface/ros/RosServerClass.h>
#include <CartesianInterface/ProblemDescription.h>


using namespace XBot::Cartesian;

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
        auto robot = XBot::RobotInterface::getRobot(XBot::Utils::getXBotConfig());
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
        model->syncFrom(*robot, XBot::Sync::Position, XBot::Sync::MotorSide);
    }

    /* Load IK problem and solver */
    auto yaml_file = YAML::LoadFile(XBot::Utils::getXBotConfig());
    ProblemDescription ik_problem(yaml_file["CartesianInterface"]["problem_description"]);
    std::string impl_name = yaml_file["CartesianInterface"]["solver"].as<std::string>();
    
    auto sot_ik_solver = SoLib::getFactoryWithArgs<XBot::Cartesian::CartesianInterface>("Cartesian" + impl_name + ".so", 
                                                                                        impl_name + "Impl", 
                                                                                        model, ik_problem);
    
    /* Obtain class to expose ROS API */
    XBot::Cartesian::RosServerClass ros_server_class(sot_ik_solver, model);
    
    ros::Rate loop_rate(100);
    Eigen::VectorXd q, qdot;
    model->getJointPosition(q);
    double time = 0.0;
    double dt = loop_rate.expectedCycleTime().toSec();
    
    while(ros::ok())
    {
        /* Update references from ros */
        ros_server_class.run();
        
        /* Solve ik */
        sot_ik_solver->update(time, dt);
        
        /* Integrate solution */
        model->getJointPosition(q);
        model->getJointVelocity(qdot);
        
        q += dt * qdot;
        
        model->setJointPosition(q);
        model->update();
        
        /* Update time and sleep */
        time += dt;
        logger->add("loop_time", loop_rate.cycleTime().toSec()*1e6);
        loop_rate.sleep();
        
        
    }
    
    logger->flush();
    
    return 0;
    
}