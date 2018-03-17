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
#include <robot_state_publisher/robot_state_publisher.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>
#include <cartesian_interface/ReferenceStamped.h>

#include <cartesian_interface/CartesianInterface.h>
#include <cartesian_interface/markers/CartesianMarker.h>
#include <thread>

namespace XBot { namespace Cartesian {
    
    /**
     * @brief The RosServerClass exposes a complete ROS API to the world.
     * For instance, the internal state of the provided CartesianInterface is
     * broadcast through ROS topics, and the user can send references to the
     * CartesianInterface by using topics/services/actions.
     */
    class RosServerClass {
        
    public:
        
        typedef std::shared_ptr<RosServerClass> Ptr;
        typedef std::shared_ptr<const RosServerClass> ConstPtr;
        
        RosServerClass(CartesianInterface::Ptr intfc, 
                       ModelInterface::ConstPtr model);
        
        /**
         * @brief This method processes callbacks (it internally calls spinOnce) and
         * also publishes to ROS.
         * @return void
         */
        void run();
        
        ModelInterface::ConstPtr getModel() const;
        
        ~RosServerClass();
        
    private:
        
        typedef actionlib::SimpleActionServer<cartesian_interface::ReachPoseAction> ActionServer;
        typedef std::shared_ptr<ActionServer> ActionServerPtr;
        
        void __generate_reach_pose_action_servers();
        void __generate_state_broadcasting();
        void __generate_online_pos_topics();
        void __generate_online_vel_topics();
        void __generate_toggle_pos_mode_services();
        void __generate_toggle_task_services();
        void __generate_reset_service();
        void __generate_rspub();
        void __generate_markers();
        
        void manage_reach_actions();
        void publish_state();
        void publish_ref_tf();
        void publish_world_tf();
        static geometry_msgs::Pose get_normalized_pose(const geometry_msgs::Pose& pose);
        
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
        ModelInterface::ConstPtr _model;
        tf::TransformBroadcaster _tf_broadcaster;
        std::unique_ptr<robot_state_publisher::RobotStatePublisher> _rspub;
        
        
        ros::NodeHandle _nh;
        
        std::vector<ActionServerPtr> _action_servers;
        std::vector<bool> _is_action_active;
        std::vector<ros::Subscriber> _pos_sub, _vel_sub;
        std::vector<ros::Publisher> _state_pub;
        std::vector<ros::ServiceServer> _toggle_pos_mode_srv, _toggle_task_srv;
        std::vector<CartesianMarker::Ptr> _markers;
        ros::ServiceServer _reset_srv;
        
        std::shared_ptr<std::thread> _marker_thread;
        
    };
    
} }


#endif








