#ifndef __CARTESIAN_INTERFACE_LOAD_CONFIG_H__
#define __CARTESIAN_INTERFACE_LOAD_CONFIG_H__

#include <xbot2_interface/ros/config_from_param.hpp>
#include <yaml-cpp/yaml.h>

namespace XBot { namespace Cartesian { namespace Utils {
    
    ConfigOptions LoadOptionsFromParamServer(std::string ns = "");
    YAML::Node LoadProblemDescription(std::string pd_name = "problem_description");
    
    
} } }


inline XBot::ConfigOptions XBot::Cartesian::Utils::LoadOptionsFromParamServer(std::string ns)
{
    auto xbot_cfg = XBot::Utils::ConfigOptionsFromParamServer(ros::NodeHandle(ns));
    ros::NodeHandle nh_private("~");
    ros::NodeHandle nh("cartesian");
    
    std::string tf_prefix = nh_private.param<std::string>("tf_prefix", "ci");
    xbot_cfg.set_parameter("tf_prefix", tf_prefix);
    
    std::string world_frame;
    if(nh_private.hasParam("world_frame_link") && nh_private.getParam("world_frame_link", world_frame))
    {
        xbot_cfg.set_parameter("world_frame_link", world_frame);
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


inline YAML::Node XBot::Cartesian::Utils::LoadProblemDescription(std::string pd_name)
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

}



#endif
