#ifndef __XBOT_CARTESIAN_ROS_ACTION_SERVER_H__
#define __XBOT_CARTESIAN_ROS_ACTION_SERVER_H__

#include <functional>

#include <rclcpp/rclcpp.hpp>

#include <cartesian_interface/srv/get_task_list.hpp>
#include <cartesian_interface/srv/reset_world.hpp>
#include <cartesian_interface/srv/set_transform.hpp>

#include <std_msgs/msg/empty.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <sensor_msgs/msg/joint_state.hpp>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_eigen/tf2_eigen.hpp>

#include <cartesian_interface/ros/utils/RobotStatePublisher.h>
#include <cartesian_interface/CartesianInterfaceImpl.h>
#include <cartesian_interface/sdk/ros/RosContext.h>
#include <cartesian_interface/sdk/ros/server_api/TaskRos.h>

namespace XBot { namespace Cartesian {

    using namespace cartesian_interface::msg;
    using namespace cartesian_interface::srv;
    using namespace geometry_msgs::msg;
    using namespace sensor_msgs::msg;
    using namespace std_srvs::srv;
    using namespace std_msgs::msg;

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
                       Options opt = Options(),
                       rclcpp::Node::SharedPtr node = rclcpp::Node::SharedPtr()
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
        void publish_solution(rclcpp::Time time);
        void publish_ref_tf(rclcpp::Time time);
        void publish_world_tf(rclcpp::Time time);

        void heartbeat_cb();

        bool reset_cb(SetBool::Request::ConstSharedPtr req,
                      SetBool::Response::SharedPtr res);

        bool task_list_cb(GetTaskList::Request::ConstSharedPtr req,
                          GetTaskList::Response::SharedPtr res);
        
        bool reset_world_cb(ResetWorld::Request::ConstSharedPtr req,
                            ResetWorld::Response::SharedPtr res);

        bool reset_base_cb(SetTransform::Request::ConstSharedPtr req,
                           SetTransform::Response::SharedPtr res);

        rclcpp::Node::SharedPtr _node;

        bool _spin_node;

        RosContext::Ptr _ros_ctx;

        std::list<ServerApi::TaskRos::Ptr> _ros_tasks;

        Options _opt;
        
        std::string _tf_prefix, _tf_prefix_slash;
        

        CartesianInterface::Ptr _ci;
        ModelInterface::ConstPtr _model;
        std::unique_ptr<RsPub> _rspub;
        rclcpp::Publisher<PointStamped>::SharedPtr _com_pub;
        rclcpp::Publisher<JointState>::SharedPtr _solution_pub;
        rclcpp::ServiceBase::SharedPtr _reset_srv,
                           _tasklist_srv, 
                           _reset_world_srv, 
                           _reset_base_srv;
        rclcpp::TimerBase::SharedPtr _heartbeat_timer;
        rclcpp::Publisher<Empty>::SharedPtr _heartbeat_pub;

        
        
    };



} }


#endif








