#ifndef __CARTESIAN_INTERFACE_LOAD_CONFIG_H__
#define __CARTESIAN_INTERFACE_LOAD_CONFIG_H__

#include <XBotInterface/ConfigOptions.h>
#include <RobotInterfaceROS/ConfigFromParam.h>

namespace XBot { namespace Cartesian { namespace Utils {
    
    ConfigOptions LoadOptionsFromConfig(YAML::Node& problem_description);
    ConfigOptions LoadOptionsFromParamServer(YAML::Node& problem_description);
    
    
} } }




inline XBot::ConfigOptions XBot::Cartesian::Utils::LoadOptionsFromConfig(YAML::Node& problem_description)
{
    std::string xbot_path_to_cfg = XBot::Utils::getXBotConfig();
    if(xbot_path_to_cfg == "")
    {
        throw std::invalid_argument("Unable to obtain path to xbot config: did you call set_xbot_config?");
    }
    auto xbot_cfg = XBot::ConfigOptions::FromConfigFile(xbot_path_to_cfg);
    
    YAML::Node ci_node = YAML::LoadFile(xbot_path_to_cfg)["CartesianInterface"]; 
    if(!ci_node)
    {
        throw std::invalid_argument("Unable to find mandatory node CartesianInterface");
    }
    
    
    if(ci_node["solver"])
    {
        std::string solver = ci_node["solver"].as<std::string>();
        xbot_cfg.set_parameter("solver", solver);
    }
    
    if(ci_node["joint_blacklist"])
    {
        std::vector<std::string> joint_blacklist = ci_node["joint_blacklist"].as<std::vector<std::string>>();
        xbot_cfg.set_parameter("joint_blacklist", joint_blacklist);
    }
    
    if(ci_node["velocity_whitelist"])
    {
        std::vector<std::string> velocity_whitelist = ci_node["velocity_whitelist"].as<std::vector<std::string>>();
        xbot_cfg.set_parameter("velocity_whitelist", velocity_whitelist);
    }
    
    if(ci_node["world_frame"])
    {
        std::string world_frame = ci_node["world_frame"].as<std::string>();
        xbot_cfg.set_parameter("world_frame", world_frame);
    }
    
    if(ci_node["problem_description"])
    {
        problem_description = ci_node["problem_description"];
    }
    else
    {
        throw std::invalid_argument("Unable to find mandatory node CartesianInterface/problem_description");
    }
    
    return xbot_cfg;
    
}

inline XBot::ConfigOptions XBot::Cartesian::Utils::LoadOptionsFromParamServer(YAML::Node& problem_description)
{
    auto xbot_cfg = XBot::ConfigOptionsFromParamServer();
    ros::NodeHandle nh_private("~");
    ros::NodeHandle nh("cartesian");
    
    std::string world_frame;
    if(nh_private.hasParam("world_frame") && nh_private.getParam("world_frame", world_frame))
    {
        xbot_cfg.set_parameter("world_frame", world_frame);
    }
    
    std::string solver;
    if(nh_private.hasParam("solver") && nh_private.getParam("solver", solver))
    {
        xbot_cfg.set_parameter("solver", solver);
    }
    
    std::string problem_description_string;
    if(!nh.hasParam("problem_description") || !nh.getParam("problem_description", problem_description_string))
    {
        throw std::runtime_error("problem_description parameter missing");
    }
    else
    {
        problem_description = YAML::Load(problem_description_string);
    }
    
    std::vector<std::string> joint_blacklist;
    if(nh.hasParam("joint_blacklist") && nh.getParam("joint_blacklist", joint_blacklist))
    {
        xbot_cfg.set_parameter("joint_blacklist", joint_blacklist);
    }
    
    std::vector<std::string> velocity_whitelist;
    if(nh.hasParam("velocity_whitelist") && nh.getParam("velocity_whitelist", velocity_whitelist))
    {
        xbot_cfg.set_parameter("velocity_whitelist", velocity_whitelist);
    }
    
    std::string js_msg_type = nh_private.param<std::string>("jointstate_message_type", "AdvrJointStateMessage");
    std::string ctrl_msg_type = nh_private.param<std::string>("control_message_type", "AdvrCommandMessage");
    
    xbot_cfg.set_parameter("jointstate_message_type", js_msg_type);
    xbot_cfg.set_parameter("control_message_type", ctrl_msg_type);
    
    return xbot_cfg;
}



#endif