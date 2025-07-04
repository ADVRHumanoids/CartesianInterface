#ifndef __XBOT_CARTESIAN_INTERFACE_RosClient_H__
#define __XBOT_CARTESIAN_INTERFACE_RosClient_H__

#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <cartesian_interface/CartesianInterfaceImpl.h>

#include <cartesian_interface_ros/srv/load_controller.hpp>

namespace XBot { namespace Cartesian {

struct RosInitializer
{
    RosInitializer(std::string ns);

    rclcpp::Node::SharedPtr node();
    void callAvailable();

private:

    rclcpp::CallbackGroup::SharedPtr _ros2_cbg;
    rclcpp::Node::SharedPtr _node;
    std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> _exec;
};

class RosClient : public CartesianInterfaceImpl,
        private virtual RosInitializer
{

public:

    CARTESIO_DECLARE_SMART_PTR(RosClient)

    friend std::ostream& operator<<(std::ostream& os, const RosClient& r);

    RosClient(std::string ns = "cartesian");

    void set_async_mode(bool async);

    //        virtual bool reset();

    bool setVelocityReference(const std::string& end_effector,
                              const Eigen::Vector6d& base_vel_ref,
                              const std::string& base_frame);

    virtual bool resetWorld(const Eigen::Affine3d& w_T_new_world);
    bool resetWorld(const std::string& ee_name);

    bool setWayPoints(const std::string& end_effector,
                      const Trajectory::WayPointVector& way_points,
                      bool incremental
                      );

    //        virtual bool reset(double time);

    void loadController(const std::string& controller_name,
                        const std::string& problem_description_name = "",
                        const std::string& problem_description_string = "",
                        const bool force_reload = true);

    bool waitReachCompleted(const std::string& ee_name, double timeout_sec = 0);

    //        virtual bool abort(const std::string& end_effector);
	
	bool setStiffnessTransition(const std::string& end_effector,
                      const Interpolator<Eigen::Matrix6d>::WayPointVector & way_points);

    bool waitStiffnessTransitionCompleted(const std::string& ee_name, double timeout_sec = 0);
	
	bool abortStiffnessTransition(const std::string& end_effector);

    virtual bool update(double time, double period);

    bool getPoseFromTf(const std::string& source_frame,
                       const std::string& target_frame,
                       Eigen::Affine3d& t_T_s);

    std::string _ns;

private:
    
    std::shared_ptr<tf2_ros::TransformListener> _listener;
    std::unique_ptr<tf2_ros::Buffer> _tf_buffer;
    rclcpp::Client<cartesian_interface_ros::srv::LoadController>::SharedPtr _load_ctrl_srv;

};

std::ostream& operator<<(std::ostream& os, const RosClient& r);

} }

#endif















