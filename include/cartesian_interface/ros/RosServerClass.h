#ifndef __XBOT_CARTESIAN_ROS_ACTION_SERVER_H__
#define __XBOT_CARTESIAN_ROS_ACTION_SERVER_H__

#include <functional>

#include <ros/ros.h>
#include <ros/callback_queue.h>

#include <eigen_conversions/eigen_msg.h>
#include <cartesian_interface/GetTaskList.h>
#include <cartesian_interface/ResetWorld.h>
#include <std_srvs/Empty.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>
#include <sensor_msgs/JointState.h>

#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>
#include <cartesian_interface/utils/RobotStatePublisher.h>

#include <cartesian_interface/CartesianInterfaceImpl.h>

#include "ros/RosContext.h"
#include "ros/server_api/TaskRos.h"

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
            std::string ros_namespace;
            
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

        typedef Utils::RobotStatePublisher RsPub;


//        void init_reach_pose_action_servers();
        void init_state_broadcasting();
//        void init_online_pos_topics();
//        void init_online_vel_topics();
//        void init_interaction_topics();
//        void init_interaction_srvs();
//        void init_task_info_services();
        void init_reset_service();
        void init_rspub();
//        void init_task_info_setters();
        void init_task_list_service();
//        void init_postural_task_topics_and_services();
//        void init_update_param_services();
        void init_reset_world_service();
        void init_heartbeat_pub();
        void init_load_ros_task_api();

//        void manage_reach_actions();
//        void publish_task_info();
//        void publish_posture_state(ros::Time time);
//        void publish_state(ros::Time time);
        void publish_solution(ros::Time time);
        void publish_ref_tf(ros::Time time);
        void publish_world_tf(ros::Time time);

//        void online_position_reference_cb(const geometry_msgs::PoseStampedConstPtr& msg,
//                                          const std::string& ee_name);
        
//        void online_impedance_reference_cb(const cartesian_interface::Impedance6ConstPtr& msg,
//                                             const std::string& ee_name);
        
//        void online_force_reference_cb(const geometry_msgs::WrenchStampedConstPtr& msg,
//                                             const std::string& ee_name);
        
//        void online_posture_reference_cb(const sensor_msgs::JointStateConstPtr& msg);

//        void online_velocity_reference_cb(const geometry_msgs::TwistStampedConstPtr& msg,
//                                          const std::string& ee_name);

        void heartbeat_cb(const ros::TimerEvent& ev);

//        bool get_task_info_cb(cartesian_interface::GetTaskInfoRequest& req,
//                            cartesian_interface::GetTaskInfoResponse& res,
//                            const std::string& ee_name);
        
//        bool get_impedance_cb(cartesian_interface::GetImpedanceRequest& req,
//                            cartesian_interface::GetImpedanceResponse& res,
//                            const std::string& ee_name);
        
//        bool set_task_info_cb(cartesian_interface::SetTaskInfoRequest& req,
//                            cartesian_interface::SetTaskInfoResponse& res,
//                            const std::string& ee_name);

        bool reset_cb(std_srvs::SetBoolRequest& req,
                      std_srvs::SetBoolResponse& res);
        
//        bool reset_params_cb(std_srvs::TriggerRequest& req,
//                             std_srvs::TriggerResponse& res);
        
//        bool reset_posture_cb(std_srvs::TriggerRequest& req,
//                              std_srvs::TriggerResponse& res);
        
        bool task_list_cb(cartesian_interface::GetTaskListRequest& req, 
                          cartesian_interface::GetTaskListResponse& res);
        
        bool reset_world_cb(cartesian_interface::ResetWorldRequest& req, 
                            cartesian_interface::ResetWorldResponse& res);

        ros::CallbackQueue _cbk_queue;
        ros::NodeHandle _nh;

        RosContext::Ptr _ros_ctx;

        std::list<ServerApi::TaskRos::Ptr> _ros_tasks;

        Options _opt;
        
        std::string _tf_prefix, _tf_prefix_slash;
        

        CartesianInterface::Ptr _ci;
        ModelInterface::ConstPtr _model;
        tf::TransformBroadcaster _tf_broadcaster;
        std::unique_ptr<RsPub> _rspub;
        ros::Publisher _com_pub, _solution_pub;
        ros::ServiceServer _reset_srv, 
                           _tasklist_srv, 
                           _reset_world_srv;
        ros::Timer _heartbeat_timer;
        ros::Publisher _heartbeat_pub;

        
        
    };



} }


#endif








