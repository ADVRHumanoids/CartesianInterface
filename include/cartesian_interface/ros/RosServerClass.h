#ifndef __XBOT_CARTESIAN_ROS_ACTION_SERVER_H__
#define __XBOT_CARTESIAN_ROS_ACTION_SERVER_H__

#include <functional>

#include <ros/ros.h>
#include <ros/callback_queue.h>

#include <eigen_conversions/eigen_msg.h>
#include <cartesian_interface/ros/RosEnabled.h>
#include <cartesian_interface/ReachPoseAction.h>
#include <cartesian_interface/GetTaskInfo.h>
#include <cartesian_interface/SetTaskInfo.h>
#include <cartesian_interface/GetTaskList.h>
#include <cartesian_interface/ResetWorld.h>
#include <cartesian_interface/Interaction.h>
#include <cartesian_interface/GetImpedance.h>
#include <actionlib/server/simple_action_server.h>
#include <std_srvs/Empty.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/JointState.h>

#ifndef XBOT_RSPUB
    #include <robot_state_publisher/robot_state_publisher.h>
#else
    #include <robot_state_publisher_advr/robot_state_publisher_advr.h>
#endif

#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>

#include <cartesian_interface/CartesianInterfaceImpl.h>

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
        
        struct Options
        {
            std::string tf_prefix;
            
            Options();
        };

        RosServerClass(CartesianInterface::Ptr intfc,
                       ModelInterface::ConstPtr model, 
                       Options opt = Options()
                      );

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
        
#ifndef XBOT_RSPUB
        typedef robot_state_publisher::RobotStatePublisher RsPub;
#else
        typedef robot_state_publisher_advr::RobotStatePublisher RsPub;
#endif

        void __generate_reach_pose_action_servers();
        void __generate_state_broadcasting();
        void __generate_online_pos_topics();
        void __generate_online_vel_topics();
        void __generate_interaction_topics();
        void __generate_interaction_srvs();
        void __generate_task_info_services();
        void __generate_reset_service();
        void __generate_rspub();
        void __generate_task_info_setters();
        void __generate_task_list_service();
        void __generate_postural_task_topics_and_services();
        void __generate_update_param_services();
        void __generate_reset_world_service();

        void manage_reach_actions();
        void publish_posture_state(ros::Time time);
        void publish_state(ros::Time time);
        void publish_solution(ros::Time time);
        void publish_ref_tf(ros::Time time);
        void publish_world_tf(ros::Time time);
        static geometry_msgs::Pose get_normalized_pose(const geometry_msgs::Pose& pose);

        void online_position_reference_cb(const geometry_msgs::PoseStampedConstPtr& msg,
                                          const std::string& ee_name);
        
        void online_interaction_reference_cb(const cartesian_interface::InteractionConstPtr& msg,
                                             const std::string& ee_name);
        
        void online_posture_reference_cb(const sensor_msgs::JointStateConstPtr& msg);

        void online_velocity_reference_cb(const geometry_msgs::TwistStampedConstPtr& msg,
                                          const std::string& ee_name);

        bool get_task_info_cb(cartesian_interface::GetTaskInfoRequest& req,
                            cartesian_interface::GetTaskInfoResponse& res,
                            const std::string& ee_name);
        
        bool get_impedance_cb(cartesian_interface::GetImpedanceRequest& req,
                            cartesian_interface::GetImpedanceResponse& res,
                            const std::string& ee_name);
        
        bool set_task_info_cb(cartesian_interface::SetTaskInfoRequest& req,
                            cartesian_interface::SetTaskInfoResponse& res,
                            const std::string& ee_name);

        bool reset_cb(std_srvs::SetBoolRequest& req,
                      std_srvs::SetBoolResponse& res);
        
        bool reset_params_cb(std_srvs::TriggerRequest& req,
                             std_srvs::TriggerResponse& res);
        
        bool reset_posture_cb(std_srvs::TriggerRequest& req,
                              std_srvs::TriggerResponse& res);
        
        bool task_list_cb(cartesian_interface::GetTaskListRequest& req, 
                          cartesian_interface::GetTaskListResponse& res);
        
        bool reset_world_cb(cartesian_interface::ResetWorldRequest& req, 
                            cartesian_interface::ResetWorldResponse& res);

        Options _opt;
        
        std::string _tf_prefix, _tf_prefix_slash;
        
        ros::NodeHandle _nh;
        ros::CallbackQueue _cbk_queue;

        CartesianInterface::Ptr _cartesian_interface;
        RosEnabled::Ptr _ros_enabled_ci;
        ModelInterface::ConstPtr _model;
        tf::TransformBroadcaster _tf_broadcaster;
        std::unique_ptr<RsPub> _rspub;

        std::vector<ActionServerPtr> _action_servers;
        std::vector<bool> _is_action_active;
        std::vector<ros::Subscriber> _pos_sub, _vel_sub, _interaction_sub;
        std::vector<ros::Publisher> _state_pub;
        ros::Publisher _com_pub, _posture_pub, _solution_pub;
        ros::Subscriber _posture_sub;
        std::vector<ros::ServiceServer> _get_task_info_srv, _set_task_info_srv, _impedance_srv;
        ros::ServiceServer _reset_srv, 
                           _tasklist_srv, 
                           _reset_posture_srv, 
                           _update_limits_srv, 
                           _reset_world_srv;

        
        
    };

} }


#endif








