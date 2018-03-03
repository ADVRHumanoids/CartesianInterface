#ifndef __XBOT_CARTESIAN_ROS_ACTION_SERVER_H__
#define __XBOT_CARTESIAN_ROS_ACTION_SERVER_H__

#include <functional>

#include <ros/ros.h>
#include <eigen_conversions/eigen_msg.h>
#include <cartesian_interface/ReachPoseAction.h>
#include <actionlib/server/simple_action_server.h>
#include <std_srvs/Empty.h>
#include <std_srvs/SetBool.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <cartesian_interface/ReferenceStamped.h>

#include <CartesianInterface/CartesianInterface.h>

namespace XBot { namespace Cartesian {
    
    class RosServerClass {
        
    public:
        
        RosServerClass(CartesianInterface::Ptr intfc);
        
        
    private:
        
        typedef actionlib::SimpleActionServer<cartesian_interface::ReachPoseAction> ActionServer;
        typedef std::shared_ptr<ActionServer> ActionServerPtr;
        
        void __generate_reach_pose_action_servers();
        void __generate_online_pos_topics();
        void __generate_online_vel_topics();
        void __generate_toggle_pos_mode_services();
        void __generate_toggle_task_services();
        void __generate_reset_service();
        
        
        void online_position_reference_cb(const geometry_msgs::PoseStampedConstPtr& msg, 
                                           const std::string& ee_name);
        
        void online_velocity_reference_cb(const geometry_msgs::TwistStampedConstPtr& msg, 
                                           const std::string& ee_name);
        
        bool toggle_position_mode_cb(std_srvs::SetBoolRequest& req, 
                                     std_srvs::SetBoolResponse& res, 
                                     const std::string& ee_name);
        
        bool toggle_task_cb(std_srvs::SetBoolRequest& req, 
                            std_srvs::SetBoolResponse& res, 
                            const std::string& ee_name);
        
        bool reset_cb(std_srvs::SetBoolRequest& req, 
                      std_srvs::SetBoolResponse& res);
        
        
        CartesianInterface::Ptr _cartesian_interface;
        
        ros::NodeHandle _nh;
        
        std::vector<ActionServerPtr> _action_servers;
        std::vector<ros::Subscriber> _pos_sub, _vel_sub;
        std::vector<ros::ServiceServer> _toggle_pos_mode_srv, _toggle_task_srv;
        ros::ServiceServer _reset_srv;
        
        
        
    };
    
} }


#endif

using namespace XBot::Cartesian;

void RosServerClass::online_position_reference_cb(const geometry_msgs::PoseStampedConstPtr& msg, 
                                                  const std::string& ee_name)
{
    
    Eigen::Affine3d T;
    tf::poseMsgToEigen(msg->pose, T);
    
    _cartesian_interface->setPoseReference(ee_name, T);
}


void RosServerClass::__generate_online_pos_topics()
{
    for(std::string ee_name : _cartesian_interface->getTaskList())
    {
        std::string topic_name = "/xbotcore/cartesian/" + ee_name + "/reference";
        
        auto cb = std::bind(&RosServerClass::online_position_reference_cb, this, std::placeholders::_1, ee_name);
        
        ros::Subscriber sub = _nh.subscribe<geometry_msgs::PoseStamped>(topic_name, 
                                                                        1, cb);
    }
}

void RosServerClass::__generate_reach_pose_action_servers()
{
    for(std::string ee_name : _cartesian_interface->getTaskList())
    {
        std::string action_name = "/xbotcore/cartesian/" + ee_name + "/reach";

        _action_servers.emplace_back( new ActionServer(_nh, action_name, false) );
        
        _action_servers.back()->start();
    }
}





