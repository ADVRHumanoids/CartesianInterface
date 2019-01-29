#ifndef __CARTESIAN_INTERFACE_LOAD_CONFIG_H__
#define __CARTESIAN_INTERFACE_LOAD_CONFIG_H__

#include <XBotInterface/ConfigOptions.h>
#include <RobotInterfaceROS/ConfigFromParam.h>

namespace XBot { namespace Cartesian { namespace Utils {
    
    enum class LoadFrom { CONFIG, PARAM };
    
    ConfigOptions LoadOptionsFromConfig();
    ConfigOptions LoadOptionsFromParamServer();
    ConfigOptions LoadOptions(LoadFrom options_source);
    YAML::Node LoadProblemDescription(LoadFrom description_source, 
                                      std::string pd_name = "problem_description");
    
    
} } }


XBot::ConfigOptions XBot::Cartesian::Utils::LoadOptions(XBot::Cartesian::Utils::LoadFrom options_source)
{
    switch(options_source)
    {
        case LoadFrom::CONFIG:
            return LoadOptionsFromConfig();
            break;
            
        case LoadFrom::PARAM:
            return LoadOptionsFromParamServer();
            break;
            
        default:
            throw std::runtime_error("Invalid options source");
            
    }
    
}

namespace
{
    YAML::Node GetCiYamlNode()
    {
        std::string xbot_path_to_cfg = XBot::Utils::getXBotConfig();
        if(xbot_path_to_cfg == "")
        {
            throw std::invalid_argument("Unable to obtain path to xbot config: did you call set_xbot_config?");
        }
        
        YAML::Node ci_node = YAML::LoadFile(xbot_path_to_cfg)["CartesianInterface"]; 
        if(!ci_node)
        {
            throw std::invalid_argument("Unable to find mandatory node CartesianInterface");
        }
        return ci_node;
    }
}

inline XBot::ConfigOptions XBot::Cartesian::Utils::LoadOptionsFromConfig()
{
    
    YAML::Node ci_node = ::GetCiYamlNode();
    
    auto xbot_cfg = XBot::ConfigOptions::FromConfigFile(XBot::Utils::getXBotConfig());
    
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
    
    
    return xbot_cfg;
    
}

inline XBot::ConfigOptions XBot::Cartesian::Utils::LoadOptionsFromParamServer()
{
    auto xbot_cfg = XBot::ConfigOptionsFromParamServer();
    ros::NodeHandle nh_private("~");
    ros::NodeHandle nh("cartesian");
    
    std::string world_frame;
    if(nh_private.hasParam("world_frame") && nh_private.getParam("world_frame", world_frame))
    {
        xbot_cfg.set_parameter("world_frame", world_frame);
    }
    
    std::vector<double> linear(3), angular(4);
    bool has_linear = nh.hasParam("floating_base_pose/linear") && \
                        nh.getParam("floating_base_pose/linear", linear) && \
                        linear.size() == 3;
                        
    bool has_angular = nh.hasParam("floating_base_pose/angular") && \
                        nh.getParam("floating_base_pose/angular", angular) && \
                        angular.size() == 4;
                        
                        
    if(has_linear && has_angular)
    {
        Eigen::Affine3d w_T_fb;
        w_T_fb.setIdentity();
        w_T_fb.linear() = Eigen::Quaterniond(Eigen::Vector4d::Map(angular.data())).toRotationMatrix();
        w_T_fb.translation() = Eigen::Vector3d::Map(linear.data());
        xbot_cfg.set_parameter("floating_base_pose", w_T_fb);
    }
    
    std::string solver;
    if(nh_private.hasParam("solver") && nh_private.getParam("solver", solver))
    {
        xbot_cfg.set_parameter("solver", solver);
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
        
    std::map<std::string, double> tau_offset_map;
    if(nh.hasParam("torque_offset") && nh.getParam("torque_offset", tau_offset_map))
    {
        std::unordered_map<std::string, double> jmap(tau_offset_map.begin(), tau_offset_map.end());
        xbot_cfg.set_parameter("torque_offset", jmap);
    }
    
    return xbot_cfg;
}


YAML::Node XBot::Cartesian::Utils::LoadProblemDescription(XBot::Cartesian::Utils::LoadFrom description_source,
                                                          std::string pd_name)

{
    switch(description_source)
    {
        case LoadFrom::CONFIG:
        {
            YAML::Node ci_node = ::GetCiYamlNode();
            if(ci_node[pd_name])
            {
                return ci_node[pd_name];
            }
            else
            {
                throw std::invalid_argument("Unable to find mandatory node CartesianInterface/" + pd_name);
            }
            break;
        }   
        case LoadFrom::PARAM:
        {
            ros::NodeHandle nh("cartesian");
            std::string problem_description_string;
            if(!nh.hasParam(pd_name) || 
                !nh.getParam(pd_name,
                             problem_description_string))
            {
                throw std::runtime_error("problem_description '" + pd_name + "' parameter missing");
            }
            else
            {
                return YAML::Load(problem_description_string);
            }
            break;
        }   
        default:
            throw std::runtime_error("Invalid options source");
            
    }
}



#endif
