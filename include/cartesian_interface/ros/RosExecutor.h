#ifndef __XBOT_CARTESIAN_ROS_EXECUTOR_H__
#define __XBOT_CARTESIAN_ROS_EXECUTOR_H__

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>

#include <XBotInterface/RobotInterface.h>
#include <XBotInterface/SoLib.h>
#include <XBotInterface/Utils.h>

#include <cartesian_interface/CartesianInterfaceImpl.h>
#include <cartesian_interface/problem/ProblemDescription.h>
#include <cartesian_interface/ros/RosServerClass.h>
#include <cartesian_interface/utils/LoadConfig.h>

#include <cartesian_interface/srv/reset_joints.hpp>
#include <cartesian_interface/srv/load_controller.hpp>

namespace XBot {
namespace Cartesian {

using namespace cartesian_interface::msg;
using namespace cartesian_interface::srv;
using namespace geometry_msgs::msg;
using namespace sensor_msgs::msg;
using namespace std_srvs::srv;
using namespace std_msgs::msg;

class RosExecutor
{

public:

    RosExecutor(std::string ns = "cartesian");

    void spin();

    void spin_once();

    CartesianInterfaceImpl &solver();

    const ModelInterface &model();

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

    void floating_base_pose_callback(PoseStamped::ConstSharedPtr msg);

    bool loader_callback(LoadController::Request::ConstSharedPtr req,
                         LoadController::Response::SharedPtr res);

    bool reset_callback(Trigger::Request::ConstSharedPtr req,
                        Trigger::Response::SharedPtr res);

    bool reset_joints_callback(ResetJoints::Request::ConstSharedPtr req,
                               ResetJoints::Response::SharedPtr res);

    void timer_callback();

    void publish_fb_cmd_vel();

    Context::Ptr _ctx;

    rclcpp::Node::SharedPtr _node, _prnode, _cnode;

    Utils::LoadFrom _options_source;

    XBot::ConfigOptions _xbot_cfg, _xbot_cfg_robot;

    RobotInterface::Ptr _robot;
    rclcpp::Publisher<Twist>::SharedPtr _fb_pub;
    bool _visual_mode;
    ModelInterface::Ptr _model;

    Eigen::VectorXd _q, _qdot, _qddot, _tau, _tau_offset;

    std::map<std::string, CartesianInterfaceImpl::Ptr> _impl_map;
    std::vector<CartesianInterfaceImpl::Ptr> _zombies;
    CartesianInterfaceImpl::Ptr _current_impl;

    RosServerClass::Ptr _ros_api;

    rclcpp::Publisher<Empty>::SharedPtr _ctrl_changed_pub;
    rclcpp::ServiceBase::SharedPtr _loader_srv;
    rclcpp::ServiceBase::SharedPtr _reset_srv;
    rclcpp::ServiceBase::SharedPtr _reset_joints_srv;
    rclcpp::SubscriptionBase::SharedPtr _fb_sub;

    bool _fb_feedback;

    rclcpp::TimerBase::SharedPtr _loop_timer;
    double _time, _period;

    MatLogger2::Ptr _logger;
};

} // namespace Cartesian
} // namespace XBot

#endif
