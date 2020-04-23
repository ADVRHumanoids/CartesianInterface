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

#include <cartesian_interface/sdk/ros/RosContext.h>
#include <cartesian_interface/sdk/ros/server_api/TaskRos.h>

namespace XBot { namespace Cartesian {

    /**
     * @brief The RosServerClass exposes a complete ROS API to the world.
     * For instance, the internal state of the provided CartesianInterface is
     * broadcast through ROS topics, and the user can send references to the
     * CartesianInterface by using topics/services/actions.
     */
    class RosServerClass {

    public:

        CARTESIO_DECLARE_SMART_PTR(RosServerClass)
        
        struct Options
        {
            std::string tf_prefix;
            std::string ros_namespace;
            bool publish_tf;
            
            Options();
        };

        RosServerClass(CartesianInterfaceImpl::Ptr intfc,
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


        void init_state_broadcasting();
        void init_reset_service();
        void init_rspub();
        void init_task_list_service();
        void init_reset_world_service();
        void init_heartbeat_pub();
        void init_load_ros_task_api();
        void publish_solution(ros::Time time);
        void publish_ref_tf(ros::Time time);
        void publish_world_tf(ros::Time time);

        void heartbeat_cb(const ros::TimerEvent& ev);
        bool reset_cb(std_srvs::SetBoolRequest& req,
                      std_srvs::SetBoolResponse& res);
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








