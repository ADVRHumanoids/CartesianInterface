#ifndef __XBOT_CARTESIAN_ROS_EXECUTOR_H__
#define __XBOT_CARTESIAN_ROS_EXECUTOR_H__

#include <chrono>

#define BOOST_RESULT_OF_USE_DECLTYPE
#include <boost/function.hpp>

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

#include <xbot2_interface/robotinterface2.h>

#include <cartesian_interface/ros/RosServerClass.h>
#include <cartesian_interface/problem/ProblemDescription.h>
#include <cartesian_interface/LoadController.h>
#include <cartesian_interface/utils/LoadConfig.h>
#include <cartesian_interface/CartesianInterfaceImpl.h>
#include <cartesian_interface/ResetJoints.h>



namespace XBot { namespace Cartesian {
   
    class RosExecutor
    {
        
    public:
        
        RosExecutor(std::string ns = "cartesian");
        
        void spin();
        void spin_once();

        CartesianInterfaceImpl& solver();
        const ModelInterface& model();
        
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

        void floating_base_pose_callback(geometry_msgs::PoseStampedConstPtr msg);
        
        bool loader_callback(cartesian_interface::LoadControllerRequest&  req, 
                             cartesian_interface::LoadControllerResponse& res);
        
        bool reset_callback(std_srvs::TriggerRequest& req, 
                            std_srvs::TriggerResponse& res);

        bool reset_joints_callback(cartesian_interface::ResetJointsRequest& req,
                                   cartesian_interface::ResetJointsResponse& res);
        
        bool pause_cartesio_callback(std_srvs::SetBoolRequest& req,
                                     std_srvs::SetBoolRequest& res);

        void timer_callback(const ros::TimerEvent& timer_ev);

        void publish_fb_cmd_vel();
        
        Context::Ptr _ctx;

        ros::CallbackQueue _fb_queue;
        ros::NodeHandle _nh, _nh_priv;
        
        XBot::ConfigOptions _xbot_cfg, _xbot_cfg_robot;
        
        RobotInterface::Ptr _robot;
        ros::Publisher _fb_pub;
        bool _visual_mode;
        ModelInterface::Ptr _model;
        
        Eigen::VectorXd _q, _qdot, _qddot, _tau, _tau_offset;
        
        std::map<std::string, CartesianInterfaceImpl::Ptr>  _impl_map;
        std::vector<CartesianInterfaceImpl::Ptr> _zombies;
        CartesianInterfaceImpl::Ptr _current_impl;
        
        RosServerClass::Ptr _ros_api;
        
        ros::Publisher _ctrl_changed_pub;
        ros::ServiceServer _loader_srv;
        ros::ServiceServer _reset_srv;
        ros::ServiceServer _reset_joints_srv;
        ros::ServiceServer _pause_ci_srv;
        ros::Subscriber _fb_sub;
        
        ros::Timer _loop_timer;
        double _time, _period;
        
        MatLogger2::Ptr _logger;

        bool _pause_command;       
        
    };

    
    
} }




#endif 
