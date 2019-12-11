#ifndef __XBOT_CARTESIAN_ROS_EXECUTOR_H__
#define __XBOT_CARTESIAN_ROS_EXECUTOR_H__

#include <chrono>

#define BOOST_RESULT_OF_USE_DECLTYPE
#include <boost/function.hpp>

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



namespace XBot { namespace Cartesian {
   
    class RosExecutor
    {
        
    public:
        
        RosExecutor(std::string ns = "cartesian");
        
        void spin();
        void spin_once();
        
        ~RosExecutor();
        
    private:
        
        void init_ros();
        void init_load_config();
        void init_load_robot();
        void init_customize_command();
        void init_load_model();
        void init_load_torque_offset();
        void init_load_world_frame();
        void init_create_loop_timer();
        
        CartesianInterfaceImpl::Ptr load_controller(std::string impl_name, 
                                                    ProblemDescription ik_problem);
        void reset_model_state();
        void load_ros_api();
        void world_frame_to_param();
        
        bool loader_callback(cartesian_interface::LoadControllerRequest&  req, 
                             cartesian_interface::LoadControllerResponse& res);
        
        bool reset_callback(std_srvs::TriggerRequest& req, 
                            std_srvs::TriggerResponse& res);
        
        void timer_callback(const ros::TimerEvent& timer_ev);
        
        
        ros::NodeHandle _nh, _nh_priv;
        
        XBot::ConfigOptions _xbot_cfg, _xbot_cfg_robot;
        Utils::LoadFrom _options_source;
        
        RobotInterface::Ptr _robot;
        ModelInterface::Ptr _model;
        
        Eigen::VectorXd _q, _qdot, _qddot, _tau, _tau_offset;
        
        std::map<std::string, CartesianInterfaceImpl::Ptr>  _impl_map;
        std::vector<CartesianInterfaceImpl::Ptr> _zombies;
        CartesianInterfaceImpl::Ptr _current_impl;
        
        RosServerClass::Ptr _ros_api;
        
        ros::Publisher _ctrl_changed_pub;
        ros::ServiceServer _loader_srv;
        ros::ServiceServer _reset_srv;
        
        ros::Timer _loop_timer;
        double _time, _period;
        
        MatLogger::Ptr _logger;
        
        
        
    };

    
    
} }




#endif 
