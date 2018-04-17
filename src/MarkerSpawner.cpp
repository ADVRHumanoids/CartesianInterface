#include <XBotInterface/RobotInterface.h>
#include <XBotInterface/Utils.h>
#include <XBotInterface/SoLib.h>

#include <cartesian_interface/CartesianInterfaceImpl.h>
#include <cartesian_interface/ProblemDescription.h>
#include <cartesian_interface/LoadController.h>
#include <cartesian_interface/markers/CartesianMarker.h>

#include <ros/ros.h>

#define BOOST_RESULT_OF_USE_DECLTYPE
#include <boost/function.hpp>
#include <chrono>



using namespace XBot::Cartesian;


int main(int argc, char **argv){
    
    /* By default, MID verbosity level */
    XBot::Logger::SetVerbosityLevel(XBot::Logger::Severity::MID);
    
    /* Init ROS node */
    ros::init(argc, argv, "xbot_cartesian_marker_spawner");
    ros::NodeHandle nh("xbotcore/cartesian");
    
    XBot::RobotInterface::Ptr robot;
    
    std::string param_name = "visual_mode";
    bool visual_mode = false;
    
    if(nh.hasParam(param_name))
    {
        nh.getParam(param_name, visual_mode);
    }

    auto model = XBot::ModelInterface::getModel(XBot::Utils::getXBotConfig());

    Eigen::VectorXd qhome;
    model->getRobotState("home", qhome);
    model->setJointPosition(qhome);
    model->update();
    
    /* Load IK problem and solver */
    auto yaml_file = YAML::LoadFile(XBot::Utils::getXBotConfig());
    ProblemDescription ik_problem(yaml_file["CartesianInterface"]["problem_description"], model);
    
    auto ci = std::make_shared<CartesianInterfaceImpl>(model, ik_problem);
    
    std::map<std::string, XBot::Cartesian::CartesianMarker::Ptr> markers;

    for(std::string ee_name : ci->getTaskList())
    {
        if(ee_name == "com")
        {
            continue;
        }
        
        std::string base_link = ci->getBaseLink(ee_name);
        base_link = base_link == "world" ? "world_odom" : base_link;
        auto marker = std::make_shared<CartesianMarker>(base_link,
                                                   ee_name,
                                                   static_cast<const urdf::Model&>(model->getUrdf()),
                                                   "ci/"
                                                  );
        
        markers[ee_name] = marker;
    }
    
    ros::spin();
};