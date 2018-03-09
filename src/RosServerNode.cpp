#include <ros/ros.h>

#include <XBotInterface/RobotInterface.h>
#include <XBotInterface/Utils.h>
#include <XBotInterface/SoLib.h>

#include <CartesianInterface/ros/RosServerClass.h>
#include <CartesianInterface/ProblemDescription.h>


using namespace XBot::Cartesian;

int main(int argc, char **argv){
    
    /* By default, MID verbosity level */
    XBot::Logger::SetVerbosityLevel(XBot::Logger::Severity::MID);
    
    /* Init ROS node */
    ros::init(argc, argv, "xbot_cartesian_server");
    ros::NodeHandle nh;
    
    /* Get model, initialize to home */
    auto model = XBot::ModelInterface::getModel(XBot::Utils::getXBotConfig());
    Eigen::VectorXd qhome;
    model->getRobotState("home", qhome);
    model->setJointPosition(qhome);
    model->update();

    /* Load IK problem and solver */
    auto yaml_file = YAML::LoadFile(XBot::Utils::getXBotConfig());
    ProblemDescription ik_problem(yaml_file["CartesianInterface"]["problem_description"]);
    
    
    auto sot_ik_solver = SoLib::getFactoryWithArgs<XBot::Cartesian::CartesianInterface>("CartesianOpenSot.so", 
                                                                                        "OpenSotImpl", 
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
        loop_rate.sleep();
        
        
    }
    
    return 0;
    
}