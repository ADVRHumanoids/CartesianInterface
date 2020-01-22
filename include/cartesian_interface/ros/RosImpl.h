#ifndef __XBOT_CARTESIAN_INTERFACE_RosClient_H__
#define __XBOT_CARTESIAN_INTERFACE_RosClient_H__

#include <string>
#include <vector>
#include <mutex>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <cartesian_interface/ReachPoseAction.h>
#include <geometry_msgs/TwistStamped.h>
#include <eigen_conversions/eigen_msg.h>
#include <cartesian_interface/TaskInfo.h>

#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

#include <cartesian_interface/CartesianInterfaceImpl.h>


namespace XBot { namespace Cartesian {

    struct RosInitializer
    {
        RosInitializer(std::string ns);

        ros::NodeHandle& nh();
        void callAvailable();

    private:

        ros::CallbackQueue _queue;
        std::unique_ptr<ros::NodeHandle> _nh;
    };
    
    class RosClient : public CartesianInterfaceImpl,
            private virtual RosInitializer
    {
        
    public:
        
        friend std::ostream& operator<<(std::ostream& os, const RosClient& r);
        
        RosClient(std::string ns = "cartesian");

//        void set_async_mode(bool async);
        
//        virtual bool reset();

//        bool setVelocityReference(const std::string& end_effector,
//                                  const Eigen::Vector6d& base_vel_ref,
//                                  const std::string& base_frame);
                            
//        virtual bool resetWorld(const Eigen::Affine3d& w_T_new_world);
//        bool resetWorld(const std::string& ee_name);
//        bool setWayPoints(const std::string& end_effector,
//                                  const Trajectory::WayPointVector& way_points,
//                                  bool incremental
//                                 );
//        virtual bool reset(double time);

        void loadController(const std::string& controller_name,
                            const std::string& problem_description_name = "",
                            const std::string& problem_description_string = "",
                            const bool force_reload = true);

        bool waitReachCompleted(const std::string& ee_name, double timeout_sec = 0);

//        virtual bool abort(const std::string& end_effector);

        virtual bool update(double time, double period);

        bool getPoseFromTf(const std::string& source_frame,
                           const std::string& target_frame,
                           Eigen::Affine3d& t_T_s);
            

    private:

        tf::TransformListener _listener;
        ros::ServiceClient _load_ctrl_srv;
        
    };

    std::ostream& operator<<(std::ostream& os, const RosClient& r);

} }

#endif















