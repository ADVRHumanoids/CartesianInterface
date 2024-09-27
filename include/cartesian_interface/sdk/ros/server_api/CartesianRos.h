#ifndef CARTESIANROSAPI_SERVERAPI_H
#define CARTESIANROSAPI_SERVERAPI_H

#include <cartesian_interface/sdk/ros/server_api/TaskRos.h>

#include <cartesian_interface/problem/Cartesian.h>

#include <actionlib/server/simple_action_server.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

#include <cartesian_interface/ReachPoseAction.h>
#include <cartesian_interface/GetCartesianTaskInfo.h>
#include <cartesian_interface/SetBaseLink.h>
#include <cartesian_interface/SetControlMode.h>
#include <cartesian_interface/SetSafetyLimits.h>
#include <cartesian_interface/CartesianTaskInfo.h>

#include <std_srvs/SetBool.h>


namespace XBot { namespace Cartesian {

class ReachActionManager
{

public:

    ReachActionManager(ros::NodeHandle nh,
                       std::string task_name,
                       CartesianTask::Ptr task);

    void run();

    ~ReachActionManager();

private:

    typedef actionlib::SimpleActionServer<cartesian_interface::ReachPoseAction> ActionServer;
    typedef std::shared_ptr<ActionServer> ActionServerPtr;

    enum class ReachActionState { IDLE, ACCEPTED, RUNNING, COMPLETED };

    void run_state_idle();
    void run_state_accepted();
    void run_state_running();
    void run_state_completed();

    ActionServerPtr _action_server;
    CartesianTask::Ptr _task;
    ReachActionState _state;
    std::string _name;

};

namespace ServerApi
{
    class CartesianRos;
}

class ServerApi::CartesianRos : public ServerApi::TaskRos,
        public virtual CartesianTaskObserver
{

public:

    CartesianRos(CartesianTask::Ptr task,
                 RosContext::Ptr context);

    bool onBaseLinkChanged() override;
    bool onControlModeChanged() override;

    virtual void run(ros::Time time) override;

protected:

private:

    void publish_ref(ros::Time time);

    void publish_task_info();

    void online_position_reference_cb(geometry_msgs::PoseStampedConstPtr  msg);

    void online_velocity_reference_cb(geometry_msgs::TwistStampedConstPtr msg);

    bool get_task_info_cb(cartesian_interface::GetCartesianTaskInfoRequest& req,
                        cartesian_interface::GetCartesianTaskInfoResponse& res);

    bool set_base_link_cb(cartesian_interface::SetBaseLinkRequest& req,
                          cartesian_interface::SetBaseLinkResponse& res);

    bool set_control_mode_cb(cartesian_interface::SetControlModeRequest& req,
                             cartesian_interface::SetControlModeResponse& res);

    bool set_safety_lims_cb(cartesian_interface::SetSafetyLimitsRequest& req,
                             cartesian_interface::SetSafetyLimitsResponse& res);

    bool use_local_velocity_reference_cb(std_srvs::SetBoolRequest& req,
                                         std_srvs::SetBoolResponse& res);



    ros::Publisher _pose_ref_pub, _vel_ref_pub, _acc_ref_pub, _task_info_pub;
    ros::Subscriber _pose_ref_sub, _vel_ref_sub;
    ros::ServiceServer _get_info_srv, _set_base_link_srv, _set_ctrl_srv, _set_safety_srv, _use_local_velocity_reference_srv;

    CartesianTask::Ptr _cart;

    std::unique_ptr<ReachActionManager> _reach_action_manager;

};

} }

#endif // CARTESIANROSAPI_H
